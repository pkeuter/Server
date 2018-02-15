#include "av_producer.h"

#include "av_input.h"

#include "../util/av_assert.h"
#include "../util/av_util.h"

#include <boost/exception/exception.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/rotate.hpp>
#include <boost/rational.hpp>

#include <common/diagnostics/graph.h>
#include <common/except.h>
#include <common/os/thread.h>
#include <common/scope_exit.h>
#include <common/timer.h>

#include <core/frame/draw_frame.h>
#include <core/frame/frame.h>
#include <core/frame/frame_factory.h>
#include <core/monitor/monitor.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#endif
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libavutil/samplefmt.h>
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <tbb/parallel_for_each.h>
#include <tbb/parallel_invoke.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>

namespace caspar { namespace ffmpeg {

using namespace std::chrono_literals;

const AVRational TIME_BASE_Q = {1, AV_TIME_BASE};

struct Frame
{
    core::draw_frame         frame;
    std::shared_ptr<AVFrame> video;
    std::shared_ptr<AVFrame> audio;
    int64_t                  pts      = AV_NOPTS_VALUE;
    int64_t                  duration = 0;
};

// TODO (fix) Handle ts discontinuities.
// TODO (feat) Forward options.

struct Decoder
{
    std::shared_ptr<AVCodecContext>       ctx;
    int64_t                               next_pts = AV_NOPTS_VALUE;
    std::queue<std::shared_ptr<AVPacket>> input;
    std::shared_ptr<AVFrame>              frame;
    bool                                  eof = false;

    Decoder() = default;

    Decoder(AVStream* stream)
    {
        const auto codec = avcodec_find_decoder(stream->codecpar->codec_id);
        if (!codec) {
            FF_RET(AVERROR_DECODER_NOT_FOUND, "avcodec_find_decoder");
        }

        ctx = std::shared_ptr<AVCodecContext>(avcodec_alloc_context3(codec),
                                              [](AVCodecContext* ptr) { avcodec_free_context(&ptr); });

        if (!ctx) {
            FF_RET(AVERROR(ENOMEM), "avcodec_alloc_context3");
        }

        FF(avcodec_parameters_to_context(ctx.get(), stream->codecpar));

        FF(av_opt_set_int(ctx.get(), "refcounted_frames", 1, 0));
        FF(av_opt_set_int(ctx.get(), "threads", 4, 0));

        ctx->pkt_timebase = stream->time_base;

        if (ctx->codec_type == AVMEDIA_TYPE_VIDEO) {
            ctx->framerate           = av_guess_frame_rate(nullptr, stream, nullptr);
            ctx->sample_aspect_ratio = av_guess_sample_aspect_ratio(nullptr, stream, nullptr);
        } else if (ctx->codec_type == AVMEDIA_TYPE_AUDIO) {
            if (!ctx->channel_layout && ctx->channels) {
                ctx->channel_layout = av_get_default_channel_layout(ctx->channels);
            }
            if (!ctx->channels && ctx->channel_layout) {
                ctx->channels = av_get_channel_layout_nb_channels(ctx->channel_layout);
            }
        }

        FF(avcodec_open2(ctx.get(), codec, nullptr));
    }
};

struct Filter
{
    std::shared_ptr<AVFilterGraph>  graph;
    AVFilterContext*                sink = nullptr;
    std::map<int, AVFilterContext*> sources;
    std::shared_ptr<AVFrame>        frame;
    bool                            eof = false;

    Filter() = default;

    Filter(std::string                    filter_spec,
           const Input&                   input,
           std::map<int, Decoder>&        streams,
           int64_t                        start_time,
           AVMediaType                    media_type,
           const core::video_format_desc& format_desc)
    {
        if (media_type == AVMEDIA_TYPE_VIDEO) {
            if (filter_spec.empty()) {
                filter_spec = "null";
            }

            filter_spec += (boost::format(",bwdif=mode=send_field:parity=auto:deint=all")).str();

            filter_spec += (boost::format(",fps=fps=%d/%d:start_time=%f") % format_desc.framerate.numerator() %
                            format_desc.framerate.denominator() % (static_cast<double>(start_time) / AV_TIME_BASE))
                               .str();
        } else if (media_type == AVMEDIA_TYPE_AUDIO) {
            if (filter_spec.empty()) {
                filter_spec = "anull";
            }

            filter_spec +=
                (boost::format(",aresample=sample_rate=%d:async=2000:first_pts=%d") % format_desc.audio_sample_rate %
                 av_rescale_q(start_time, TIME_BASE_Q, {1, format_desc.audio_sample_rate}))
                    .str();
        }

        AVFilterInOut* outputs = nullptr;
        AVFilterInOut* inputs  = nullptr;

        CASPAR_SCOPE_EXIT
        {
            avfilter_inout_free(&inputs);
            avfilter_inout_free(&outputs);
        };

        int video_input_count = 0;
        int audio_input_count = 0;
        {
            auto graph2 = avfilter_graph_alloc();
            if (!graph2) {
                FF_RET(AVERROR(ENOMEM), "avfilter_graph_alloc");
            }

            CASPAR_SCOPE_EXIT
            {
                avfilter_graph_free(&graph2);
                avfilter_inout_free(&inputs);
                avfilter_inout_free(&outputs);
            };

            FF(avfilter_graph_parse2(graph2, filter_spec.c_str(), &inputs, &outputs));

            for (auto cur = inputs; cur; cur = cur->next) {
                const auto type = avfilter_pad_get_type(cur->filter_ctx->input_pads, cur->pad_idx);
                if (type == AVMEDIA_TYPE_VIDEO) {
                    video_input_count += 1;
                } else if (type == AVMEDIA_TYPE_AUDIO) {
                    audio_input_count += 1;
                }
            }
        }

        std::vector<AVStream*> av_streams;
        for (auto n = 0U; n < input->nb_streams; ++n) {
            auto disposition = input->streams[n]->disposition;
            if (!disposition || disposition == AV_DISPOSITION_DEFAULT) {
                av_streams.push_back(input->streams[n]);
            }
        }

        if (audio_input_count == 1) {
            auto count = std::count_if(av_streams.begin(), av_streams.end(), [](auto s) {
                return s->codecpar->codec_type == AVMEDIA_TYPE_AUDIO;
            });
            
            // TODO (fix) Use some form of stream meta data to do this.
            // https://github.com/CasparCG/server/issues/833
            if (count > 1) {
                filter_spec = (boost::format("amerge=inputs=%d,") % count).str() + filter_spec;
            }
        }

        if (video_input_count == 1) {
            std::stable_sort(av_streams.begin(), av_streams.end(), [](auto lhs, auto rhs) {
                return lhs->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && lhs->codecpar->height > rhs->codecpar->height;
            });

            std::vector<AVStream*> video_av_streams;
            std::copy_if(av_streams.begin(), av_streams.end(), std::back_inserter(video_av_streams), [](auto s) {
                return s->codecpar->codec_type == AVMEDIA_TYPE_VIDEO;
            });

            // TODO (fix) Use some form of stream meta data to do this.
            // https://github.com/CasparCG/server/issues/832
            if (video_av_streams.size() >= 2 &&
                video_av_streams[0]->codecpar->height == video_av_streams[1]->codecpar->height) {
                filter_spec = "alphamerge," + filter_spec;
            }
        }

        graph = std::shared_ptr<AVFilterGraph>(avfilter_graph_alloc(),
                                               [](AVFilterGraph* ptr) { avfilter_graph_free(&ptr); });

        if (!graph) {
            FF_RET(AVERROR(ENOMEM), "avfilter_graph_alloc");
        }

        graph->nb_threads = 16;
        graph->execute    = graph_execute;

        FF(avfilter_graph_parse2(graph.get(), filter_spec.c_str(), &inputs, &outputs));

        // inputs
        {
            for (auto cur = inputs; cur; cur = cur->next) {
                const auto type = avfilter_pad_get_type(cur->filter_ctx->input_pads, cur->pad_idx);
                if (type != AVMEDIA_TYPE_VIDEO && type != AVMEDIA_TYPE_AUDIO) {
                    CASPAR_THROW_EXCEPTION(ffmpeg_error_t() << boost::errinfo_errno(EINVAL)
                                                            << msg_info_t("only video and audio filters supported"));
                }

                unsigned index = 0;

                // TODO find stream based on link name
                while (true) {
                    if (index == av_streams.size()) {
                        graph = nullptr;
                        return;
                    }
                    if (av_streams.at(index)->codecpar->codec_type == type &&
                        sources.find(static_cast<int>(index)) == sources.end()) {
                        break;
                    }
                    index++;
                }

                index = av_streams.at(index)->index;

                auto it = streams.find(index);
                if (it == streams.end()) {
                    it = streams.emplace(index, input->streams[index]).first;
                }

                auto st = it->second.ctx;

                if (st->codec_type == AVMEDIA_TYPE_VIDEO) {
                    auto args = (boost::format("video_size=%dx%d:pix_fmt=%d:time_base=%d/%d") % st->width % st->height %
                                 st->pix_fmt % st->pkt_timebase.num % st->pkt_timebase.den)
                                    .str();
                    auto name = (boost::format("in_%d") % index).str();

                    if (st->sample_aspect_ratio.num > 0 && st->sample_aspect_ratio.den > 0) {
                        args +=
                            (boost::format(":sar=%d/%d") % st->sample_aspect_ratio.num % st->sample_aspect_ratio.den)
                                .str();
                    }

                    if (st->framerate.num > 0 && st->framerate.den > 0) {
                        args += (boost::format(":frame_rate=%d/%d") % st->framerate.num % st->framerate.den).str();
                    }

                    AVFilterContext* source = nullptr;
                    FF(avfilter_graph_create_filter(
                        &source, avfilter_get_by_name("buffer"), name.c_str(), args.c_str(), nullptr, graph.get()));
                    FF(avfilter_link(source, 0, cur->filter_ctx, cur->pad_idx));
                    sources.emplace(index, source);
                } else if (st->codec_type == AVMEDIA_TYPE_AUDIO) {
                    auto args = (boost::format("time_base=%d/%d:sample_rate=%d:sample_fmt=%s:channel_layout=%#x") %
                                 st->pkt_timebase.num % st->pkt_timebase.den % st->sample_rate %
                                 av_get_sample_fmt_name(st->sample_fmt) % st->channel_layout)
                                    .str();
                    auto name = (boost::format("in_%d") % index).str();

                    AVFilterContext* source = nullptr;
                    FF(avfilter_graph_create_filter(
                        &source, avfilter_get_by_name("abuffer"), name.c_str(), args.c_str(), nullptr, graph.get()));
                    FF(avfilter_link(source, 0, cur->filter_ctx, cur->pad_idx));
                    sources.emplace(index, source);
                } else {
                    CASPAR_THROW_EXCEPTION(ffmpeg_error_t() << boost::errinfo_errno(EINVAL)
                                                            << msg_info_t("invalid filter input media type"));
                }
            }
        }

        if (media_type == AVMEDIA_TYPE_VIDEO) {
            FF(avfilter_graph_create_filter(
                &sink, avfilter_get_by_name("buffersink"), "out", nullptr, nullptr, graph.get()));

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4245)
#endif
            const AVPixelFormat pix_fmts[] = {AV_PIX_FMT_RGB24,
                                              AV_PIX_FMT_BGR24,
                                              AV_PIX_FMT_BGRA,
                                              AV_PIX_FMT_ARGB,
                                              AV_PIX_FMT_RGBA,
                                              AV_PIX_FMT_ABGR,
                                              AV_PIX_FMT_YUV444P,
                                              AV_PIX_FMT_YUV422P,
                                              AV_PIX_FMT_YUV420P,
                                              AV_PIX_FMT_YUV410P,
                                              AV_PIX_FMT_YUVA444P,
                                              AV_PIX_FMT_YUVA422P,
                                              AV_PIX_FMT_YUVA420P,
                                              AV_PIX_FMT_NONE};
            FF(av_opt_set_int_list(sink, "pix_fmts", pix_fmts, -1, AV_OPT_SEARCH_CHILDREN));
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        } else if (media_type == AVMEDIA_TYPE_AUDIO) {
            FF(avfilter_graph_create_filter(
                &sink, avfilter_get_by_name("abuffersink"), "out", nullptr, nullptr, graph.get()));
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4245)
#endif
            const AVSampleFormat sample_fmts[] = {AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_NONE};
            FF(av_opt_set_int_list(sink, "sample_fmts", sample_fmts, -1, AV_OPT_SEARCH_CHILDREN));

            // TODO Always output 8 channels and remove hack in make_frame.

            const int sample_rates[] = {format_desc.audio_sample_rate, -1};
            FF(av_opt_set_int_list(sink, "sample_rates", sample_rates, -1, AV_OPT_SEARCH_CHILDREN));
#ifdef _MSC_VER
#pragma warning(pop)
#endif
        } else {
            CASPAR_THROW_EXCEPTION(ffmpeg_error_t()
                                   << boost::errinfo_errno(EINVAL) << msg_info_t("invalid output media type"));
        }

        // output
        {
            const auto cur = outputs;

            if (!cur || cur->next) {
                CASPAR_THROW_EXCEPTION(ffmpeg_error_t() << boost::errinfo_errno(EINVAL)
                                                        << msg_info_t("invalid filter graph output count"));
            }

            if (avfilter_pad_get_type(cur->filter_ctx->output_pads, cur->pad_idx) != media_type) {
                CASPAR_THROW_EXCEPTION(ffmpeg_error_t() << boost::errinfo_errno(EINVAL)
                                                        << msg_info_t("invalid filter output media type"));
            }

            FF(avfilter_link(cur->filter_ctx, cur->pad_idx, sink, 0));
        }

        FF(avfilter_graph_config(graph.get(), nullptr));
    }
};

struct AVProducer::Impl
{
    core::monitor::state                state_;
    spl::shared_ptr<diagnostics::graph> graph_;

    const std::shared_ptr<core::frame_factory> frame_factory_;
    const core::video_format_desc              format_desc_;
    const AVRational                           format_tb_;
    const std::string                          name_;
    const std::string                          path_;

    std::vector<int> audio_cadence_ = format_desc_.audio_cadence;

    Input                  input_;
    std::map<int, Decoder> decoders_;
    Filter                 video_filter_;
    Filter                 audio_filter_;

    std::map<int, std::vector<AVFilterContext*>> sources_;

    int64_t start_    = AV_NOPTS_VALUE;
    int64_t duration_ = AV_NOPTS_VALUE;
    bool    loop_     = false;

    std::string afilter_;
    std::string vfilter_;

    mutable std::mutex      mutex_;
    std::condition_variable cond_;

    int64_t          frame_time_  = 0;
    bool             frame_flush_ = true;
    core::draw_frame frame_;

    std::mutex              buffer_mutex_;
    std::condition_variable buffer_cond_;
    std::deque<Frame>       buffer_;
    int                     buffer_capacity_ = static_cast<int>(format_desc_.fps / 2);

    std::atomic<bool> abort_request_{false};
    std::thread       thread_;

    Impl(std::shared_ptr<core::frame_factory> frame_factory,
         core::video_format_desc              format_desc,
         std::string                          name,
         std::string                          path,
         std::string                          vfilter,
         std::string                          afilter,
         boost::optional<int64_t>             start,
         boost::optional<int64_t>             duration,
         bool                                 loop)
        : frame_factory_(frame_factory)
        , format_desc_(format_desc)
        , format_tb_({format_desc.duration * format_desc.field_count, format_desc.time_scale})
        , path_(path)
        , name_(name)
        , input_(path, graph_)
        , start_(start ? av_rescale_q(*start, format_tb_, TIME_BASE_Q) : AV_NOPTS_VALUE)
        , duration_(duration ? av_rescale_q(*duration, format_tb_, TIME_BASE_Q) : AV_NOPTS_VALUE)
        , loop_(loop)
        , vfilter_(vfilter)
        , afilter_(afilter)
    {
        state_["file/name"]  = u8(name_);
        state_["file/path"]  = u8(path_);
        state_["file/time"]  = {time(), this->duration().value_or(0) / format_desc_.fps};

        diagnostics::register_graph(graph_);
        graph_->set_color("underflow", diagnostics::color(0.6f, 0.3f, 0.9f));
        graph_->set_color("frame-time", caspar::diagnostics::color(0.0f, 1.0f, 0.0f));
        graph_->set_text(u16(print()));

        if (start_ != AV_NOPTS_VALUE) {
            input_.seek(start_);
            reset(start_);
        } else {
            reset(input_.start_time().value_or(0));
        }

        thread_ = std::thread([=] {
            try {
                set_thread_name(L"[ffmpeg::av_producer]");

                boost::range::rotate(audio_cadence_, std::end(audio_cadence_) - 1);

                Frame frame;

                while (!abort_request_) {
                    {
                        std::unique_lock<std::mutex> lock(buffer_mutex_);
                        buffer_cond_.wait(lock, [&] { return buffer_.size() < buffer_capacity_ || abort_request_; });
                    }

                    if (abort_request_) {
                        break;
                    }

                    caspar::timer frame_timer;
                    CASPAR_SCOPE_EXIT
                    {
                        graph_->set_value("frame-time", frame_timer.elapsed() * format_desc_.fps * 0.5);
                    };

                    std::unique_lock<std::mutex> lock(mutex_);

                    std::atomic<int> progress{0};

                    progress.fetch_or(schedule_decoders());
                    progress.fetch_or(schedule_filters());

                    tbb::parallel_invoke(
                        [&] {
                            tbb::parallel_for_each(decoders_.begin(), decoders_.end(), [&](auto& p) {
                                progress.fetch_or(decode_frame(p.second));
                            });
                        },
                        [&] { progress.fetch_or(filter_frame(video_filter_)); },
                        [&] { progress.fetch_or(filter_frame(audio_filter_, audio_cadence_[0])); });

                    if ((!video_filter_.frame && !video_filter_.eof) || (!audio_filter_.frame && !audio_filter_.eof)) {
                        if (!progress) {
                            // TODO (perf) Avoid polling.
                            cond_.wait_for(lock, 5ms, [&] { return abort_request_.load(); });
                        }
                        // TODO (fix) Limit live polling due to bugs.
                        continue;
                    }

                    // TODO (perf) seek as soon as input is past duration or eof.
                    {
                        auto       start = start_ != AV_NOPTS_VALUE ? start_ : 0;
                        const auto eof   = (video_filter_.eof && audio_filter_.eof) ||
                                         (duration_ != AV_NOPTS_VALUE && frame.pts >= start + duration_);

                        if (eof) {
                            if (loop_) {
                                seek_internal(start_);
                            } else {
                                // TODO (perf) Avoid polling.
                                cond_.wait_for(lock, 5ms, [&] { return abort_request_.load(); });
                            }
                            // TODO (fix) Limit live polling due to bugs.
                            continue;
                        }
                    }

                    // TODO (fix)
                    // if (start_ != AV_NOPTS_VALUE && frame.pts < start_) {
                    //    seek_internal(start_);
                    //    continue;
                    //}

                    const auto start_time = input_->start_time != AV_NOPTS_VALUE ? input_->start_time : 0;

                    if (video_filter_.frame) {
                        frame.video    = std::move(video_filter_.frame);
                        const auto tb  = av_buffersink_get_time_base(video_filter_.sink);
                        const auto fr  = av_buffersink_get_frame_rate(video_filter_.sink);
                        frame.pts      = av_rescale_q(frame.video->pts, tb, TIME_BASE_Q) - start_time;
                        frame.duration = av_rescale_q(1, av_inv_q(fr), TIME_BASE_Q);
                    }

                    if (audio_filter_.frame) {
                        frame.audio    = std::move(audio_filter_.frame);
                        const auto tb  = av_buffersink_get_time_base(audio_filter_.sink);
                        const auto sr  = av_buffersink_get_sample_rate(audio_filter_.sink);
                        frame.pts      = av_rescale_q(frame.audio->pts, tb, TIME_BASE_Q) - start_time;
                        frame.duration = av_rescale_q(frame.audio->nb_samples, {1, sr}, TIME_BASE_Q);
                    }

                    frame.frame = core::draw_frame(make_frame(this, *frame_factory_, frame.video, frame.audio));

                    {
                        std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
                        buffer_.push_back(frame);
                    }

                    boost::range::rotate(audio_cadence_, std::end(audio_cadence_) - 1);
                }
            } catch (...) {
                CASPAR_LOG_CURRENT_EXCEPTION();
            }
        });
    }

    ~Impl()
    {
        abort_request_ = true;
        cond_.notify_all();
        buffer_cond_.notify_all();
        thread_.join();
    }

    bool schedule_decoders()
    {
        auto result = false;
        input_([&](std::shared_ptr<AVPacket>& packet) {
            auto min = std::numeric_limits<std::size_t>::max();
            for (auto& p : decoders_) {
                min = std::min(min, p.second.input.size());
            }

            if (min > 0) {
                return false;
            }

            if (!packet) {
                for (auto& p : decoders_) {
                    p.second.input.push(nullptr);
                }
                result = true;
            } else if (sources_.find(packet->stream_index) != sources_.end()) {
                auto it = decoders_.find(packet->stream_index);
                if (it == decoders_.end()) {
                    return true;
                }

                if (it->second.input.size() >= 512) {
                    return false;
                }

                result = true;

                it->second.input.push(std::move(packet));
            }

            return true;
        });
        return result;
    }

    bool schedule_filters()
    {
        auto result = false;

        std::vector<int> eof;

        for (auto& p : sources_) {
            auto it = decoders_.find(p.first);
            if (it == decoders_.end() || !it->second.frame) {
                continue;
            }

            auto nb_requests = 0U;
            for (auto source : p.second) {
                nb_requests = std::max(nb_requests, av_buffersrc_get_nb_failed_requests(source));
            }

            if (nb_requests == 0) {
                continue;
            }

            auto frame = std::move(it->second.frame);

            for (auto& source : p.second) {
                if (frame && !frame->data[0]) {
                    FF(av_buffersrc_close(source, frame->pts, 0));
                } else {
                    // TODO (fix) Guard against overflow?
                    FF(av_buffersrc_write_frame(source, frame.get()));
                }
                result = true;
            }

            // End Of File
            if (!frame->data[0]) {
                eof.push_back(p.first);
            }
        }

        for (auto index : eof) {
            sources_.erase(index);
        }

        return result;
    }

    bool decode_frame(Decoder& decoder)
    {
        if (decoder.frame || decoder.eof) {
            return false;
        }

        auto frame = alloc_frame();
        auto ret   = avcodec_receive_frame(decoder.ctx.get(), frame.get());

        if (ret == AVERROR(EAGAIN)) {
            if (decoder.input.empty()) {
                return false;
            }
            FF(avcodec_send_packet(decoder.ctx.get(), decoder.input.front().get()));
            decoder.input.pop();
        } else if (ret == AVERROR_EOF) {
            avcodec_flush_buffers(decoder.ctx.get());
            frame->pts       = decoder.next_pts;
            decoder.eof      = true;
            decoder.next_pts = AV_NOPTS_VALUE;
            decoder.frame    = std::move(frame);
        } else {
            FF_RET(ret, "avcodec_receive_frame");

            // TODO (fix) is this always best?
            frame->pts = frame->best_effort_timestamp;
            // TODO (fix) is this always best?
            decoder.next_pts = frame->pts + frame->pkt_duration;
            decoder.frame    = std::move(frame);
        }

        return true;
    }

    bool filter_frame(Filter& filter, int nb_samples = -1)
    {
        if (filter.frame || filter.eof) {
            return false;
        }

        if (!filter.sink || filter.sources.empty()) {
            filter.eof   = true;
            filter.frame = nullptr;
            return true;
        }

        auto frame = alloc_frame();
        auto ret   = nb_samples >= 0 ? av_buffersink_get_samples(filter.sink, frame.get(), nb_samples)
                                   : av_buffersink_get_frame(filter.sink, frame.get());

        if (ret == AVERROR(EAGAIN)) {
            return false;
        } else if (ret == AVERROR_EOF) {
            filter.eof   = true;
            filter.frame = nullptr;
            return true;
        } else {
            FF_RET(ret, "av_buffersink_get_frame");
            filter.frame = frame;
            return true;
        }
    }

    core::draw_frame prev_frame()
    {
        CASPAR_SCOPE_EXIT
        {
            graph_->set_text(u16(print()));
            state_["file/time"] = {time() / format_desc_.fps, input_.duration().value_or(0) / format_desc_.fps};
        };

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            if (!buffer_.empty() && (frame_flush_ || !frame_)) {
                frame_       = core::draw_frame::still(buffer_[0].frame);
                frame_time_  = buffer_[0].pts + buffer_[0].duration;
                frame_flush_ = false;
            }
        }

        return frame_;
    }

    core::draw_frame next_frame()
    {
        CASPAR_SCOPE_EXIT
        {
            graph_->set_text(u16(print()));
            state_["file/time"] = {time() / format_desc_.fps, duration().value_or(0) / format_desc_.fps};
        };

        core::draw_frame result;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            if (buffer_.empty() || (frame_flush_ && buffer_.size() < 4)) {
                graph_->set_tag(diagnostics::tag_severity::WARNING, "underflow");
                return core::draw_frame{};
            }

            result      = buffer_[0].frame;
            frame_      = core::draw_frame::still(buffer_[0].frame);
            frame_time_ = buffer_[0].pts + buffer_[0].duration;
            buffer_.pop_front();

            frame_flush_ = false;
        }
        buffer_cond_.notify_all();

        return result;
    }

    void seek(int64_t time)
    {
        // TODO (fix) Validate input.

        {
            std::lock_guard<std::mutex> lock(mutex_);

            {
                std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
                buffer_.clear();
            }
            buffer_cond_.notify_all();

            seek_internal(av_rescale_q(time, format_tb_, TIME_BASE_Q));
        }

        cond_.notify_all();
    }

    int64_t time() const
    {
        // TODO (fix) How to handle NOPTS case?
        return frame_time_ != AV_NOPTS_VALUE ? av_rescale_q(frame_time_, TIME_BASE_Q, format_tb_) : 0;
    }

    void loop(bool loop)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            loop_ = loop;
        }
        cond_.notify_all();
    }

    bool loop() const { return loop_; }

    void start(int64_t start)
    {
        // TODO (fix) Validate input.

        {
            std::lock_guard<std::mutex> lock(mutex_);
            start_ = av_rescale_q(start, format_tb_, TIME_BASE_Q);
        }
        cond_.notify_all();
    }

    boost::optional<int64_t> start() const
    {
        return start_ != AV_NOPTS_VALUE ? av_rescale_q(start_, TIME_BASE_Q, format_tb_) : boost::optional<int64_t>();
    }

    void duration(int64_t duration)
    {
        // TODO (fix) Validate input.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            duration_ = av_rescale_q(duration, format_tb_, TIME_BASE_Q);
            input_.paused(false);
        }
        cond_.notify_all();
    }

    boost::optional<int64_t> duration() const
    {
        if (!input_.duration()) {
            return boost::none;
        }
        return av_rescale_q(*input_.duration(), TIME_BASE_Q, format_tb_);
    }

  private:
    std::string print() const
    {
        std::ostringstream str;
        str << std::fixed << std::setprecision(4) << "ffmpeg[" << name_ << "|"
            << av_q2d({static_cast<int>(time()) * format_tb_.num, format_tb_.den}) << "/"
            << av_q2d({static_cast<int>(duration().value_or(0LL)) * format_tb_.num, format_tb_.den}) << "]";
        return str.str();
    }

    void seek_internal(int64_t time)
    {
        time = time != AV_NOPTS_VALUE ? time : 0;
        time = time + input_.start_time().value_or(0);

        // TODO (fix) Dont seek if time is close future.
        input_.seek(time);
        input_.paused(false);
        frame_flush_ = true;

        for (auto& p : decoders_) {
            reset_decoder(p.second);
        }

        reset(time);
    }

    void reset_decoder(Decoder& decoder)
    {
        avcodec_flush_buffers(decoder.ctx.get());
        decoder.next_pts = AV_NOPTS_VALUE;
        decoder.frame    = nullptr;
        decoder.eof      = false;
        decoder.input    = decltype(decoder.input){};
    }

    void reset(int64_t start_time)
    {
        video_filter_ = Filter(vfilter_, input_, decoders_, start_time, AVMEDIA_TYPE_VIDEO, format_desc_);
        audio_filter_ = Filter(afilter_, input_, decoders_, start_time, AVMEDIA_TYPE_AUDIO, format_desc_);

        sources_.clear();
        for (auto& p : video_filter_.sources) {
            sources_[p.first].push_back(p.second);
        }
        for (auto& p : audio_filter_.sources) {
            sources_[p.first].push_back(p.second);
        }

        // Flush unused inputs.
        for (auto& p : decoders_) {
            if (sources_.find(p.first) == sources_.end()) {
                reset_decoder(p.second);
            }
        }
    }
};

AVProducer::AVProducer(std::shared_ptr<core::frame_factory> frame_factory,
                       core::video_format_desc              format_desc,
                       std::string                          name,
                       std::string                          path,
                       boost::optional<std::string>         vfilter,
                       boost::optional<std::string>         afilter,
                       boost::optional<int64_t>             start,
                       boost::optional<int64_t>             duration,
                       boost::optional<bool>                loop)
    : impl_(new Impl(std::move(frame_factory),
                     std::move(format_desc),
                     std::move(name),
                     std::move(path),
                     std::move(vfilter.get_value_or("")),
                     std::move(afilter.get_value_or("")),
                     std::move(start),
                     std::move(duration),
                     std::move(loop.get_value_or(false))))
{
}

core::draw_frame AVProducer::next_frame() { return impl_->next_frame(); }

core::draw_frame AVProducer::prev_frame() { return impl_->prev_frame(); }

AVProducer& AVProducer::seek(int64_t time)
{
    impl_->seek(time);
    return *this;
}

AVProducer& AVProducer::loop(bool loop)
{
    impl_->loop(loop);
    return *this;
}

bool AVProducer::loop() const { return impl_->loop(); }

AVProducer& AVProducer::start(int64_t start)
{
    impl_->start(start);
    return *this;
}

int64_t AVProducer::time() const { return impl_->time(); }

int64_t AVProducer::start() const { return impl_->start().value_or(0); }

AVProducer& AVProducer::duration(int64_t duration)
{
    impl_->duration(duration);
    return *this;
}

int64_t AVProducer::duration() const { return impl_->duration().value_or(std::numeric_limits<int64_t>::max()); }

const core::monitor::state& AVProducer::state() const { return impl_->state_; }

}} // namespace caspar::ffmpeg
