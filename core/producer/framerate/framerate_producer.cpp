/*
* Copyright (c) 2011 Sveriges Television AB <info@casparcg.com>
*
* This file is part of CasparCG (www.casparcg.com).
*
* CasparCG is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CasparCG is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
*
* Author: Helge Norberg, helge.norberg@svt.se
*/

#include "../../StdAfx.h"

#include "framerate_producer.h"

#include "../frame_producer.h"
#include "../../frame/audio_channel_layout.h"
#include "../../frame/draw_frame.h"
#include "../../frame/frame.h"
#include "../../frame/frame_transform.h"
#include "../../frame/pixel_format.h"
#include "../../monitor/monitor.h"

#include <common/future.h>

#include <functional>
#include <queue>
#include <future>

namespace caspar { namespace core {

draw_frame drop_and_skip(const draw_frame& source, const draw_frame&, const boost::rational<int64_t>&)
{
	return source;
}

// Blends next frame with current frame when the distance is not 0.
// Completely sharp when distance is 0 but blurry when in between.
draw_frame blend(const draw_frame& source, const draw_frame& destination, const boost::rational<int64_t>& distance)
{
	if (destination == draw_frame::empty())
		return source;

	auto under					= source;
	auto over					= destination;
	double float_distance		= static_cast<double>(distance.numerator()) / static_cast<double>(distance.denominator());

	under.transform().image_transform.is_mix	= true;
	under.transform().image_transform.opacity	= 1 - float_distance;
	over.transform().image_transform.is_mix		= true;
	over.transform().image_transform.opacity	= float_distance;

	return draw_frame::over(under, over);
}

// Blends a moving window with a width of 1 frame duration.
// * A distance of 0.0 gives 50% previous, 50% current and 0% next.
// * A distance of 0.5 gives 25% previous, 50% current and 25% next.
// * A distance of 0.75 gives 12.5% previous, 50% current and 37.5% next.
// This is blurrier than blend, but gives a more even bluriness, instead of sharp, blurry, sharp, blurry.
struct blend_all
{
	draw_frame previous_frame	= draw_frame::empty();
	draw_frame last_source		= draw_frame::empty();
	draw_frame last_destination	= draw_frame::empty();

	draw_frame operator()(const draw_frame& source, const draw_frame& destination, const boost::rational<int64_t>& distance)
	{
		if (last_source != draw_frame::empty() && last_source != source)
		{
			if (last_destination == source)
				previous_frame = last_source;
			else // A two frame jump
				previous_frame = last_destination;
		}

		last_source			= source;
		last_destination	= destination;

		bool has_previous = previous_frame != draw_frame::empty();

		if (!has_previous)
			return blend(source, destination, distance);

		auto middle											= last_source;
		auto next_frame										= destination;
		previous_frame.transform().image_transform.is_mix	= true;
		middle.transform().image_transform.is_mix			= true;
		next_frame.transform().image_transform.is_mix		= true;

		double float_distance								= static_cast<double>(distance.numerator()) / static_cast<double>(distance.denominator());
		previous_frame.transform().image_transform.opacity	= std::max(0.0, 0.5 - float_distance * 0.5);
		middle.transform().image_transform.opacity			= 0.5;
		next_frame.transform().image_transform.opacity		= 1.0 - previous_frame.transform().image_transform.opacity - middle.transform().image_transform.opacity;

		std::vector<draw_frame> combination { previous_frame, middle, next_frame };

		return draw_frame(std::move(combination));
	}
};

struct audio_extractor : public frame_visitor
{
	std::function<void(const const_frame& frame)> on_frame_;

	audio_extractor(std::function<void(const const_frame& frame)> on_frame)
		: on_frame_(std::move(on_frame))
	{
	}

	void push(const frame_transform& transform) override	{ }
	void pop() override										{ }
	void visit(const const_frame& frame) override
	{
		if (!frame.audio_data().empty())
			on_frame_(frame);
	}
};

class framerate_producer : public frame_producer_base
{
	spl::shared_ptr<frame_producer>						source_;
	boost::rational<int>								source_framerate_;
	audio_channel_layout								source_channel_layout_		= audio_channel_layout::invalid();
	boost::rational<int>								destination_framerate_;
	field_mode											destination_fieldmode_;
	std::vector<int>									destination_audio_cadence_;
	boost::rational<std::int64_t>						speed_;
	std::function<draw_frame (
			const draw_frame& source,
			const draw_frame& destination,
			const boost::rational<int64_t>& distance)>	interpolator_				= drop_and_skip;
	
	boost::rational<std::int64_t>						current_frame_number_		= 0;
	draw_frame											previous_frame_				= draw_frame::empty();
	draw_frame											next_frame_					= draw_frame::empty();
	mutable_audio_buffer								audio_samples_;

	unsigned int										output_repeat_				= 0;
	unsigned int										output_frame_				= 0;
public:
	framerate_producer(
			spl::shared_ptr<frame_producer> source,
			boost::rational<int> source_framerate,
			boost::rational<int> destination_framerate,
			field_mode destination_fieldmode,
			std::vector<int> destination_audio_cadence)
		: source_(std::move(source))
		, source_framerate_(std::move(source_framerate))
		, destination_framerate_(std::move(destination_framerate))
		, destination_fieldmode_(destination_fieldmode)
		, destination_audio_cadence_(std::move(destination_audio_cadence))
	{
		// Coarse adjustment to correct fps family (23.98 - 30 vs 47.95 - 60)
		if (destination_fieldmode_ != field_mode::progressive)	// Interlaced output
		{
			auto diff_double	= boost::abs(source_framerate_ - destination_framerate_ * 2);
			auto diff_keep		= boost::abs(source_framerate_ - destination_framerate_);

			if (diff_double < diff_keep)						// Double rate interlaced
			{
				destination_framerate_ *= 2;
			}
			else												// Progressive non interlaced
			{
				destination_fieldmode_ = field_mode::progressive;
			}
		}
		else													// Progressive
		{
			auto diff_halve	= boost::abs(source_framerate_ * 2	- destination_framerate_);
			auto diff_keep	= boost::abs(source_framerate_		- destination_framerate_);

			if (diff_halve < diff_keep)							// Repeat every frame two times
			{
				destination_framerate_	/= 2;
				output_repeat_			= 2;
			}
		}

		speed_ = boost::rational<int64_t>(source_framerate_ / destination_framerate_);

		// drop_and_skip will only be used by default for exact framerate multiples (half, same and double)
		// for all other framerates a frame interpolator will be chosen.
		if (speed_ != 1 && speed_ * 2 != 1 && speed_ != 2)
		{
			if (source_framerate_ > 47)		// The bluriness of blend_all is acceptable on high framerates.
				interpolator_ = blend_all();
			else							// blend_all is mostly too blurry on low framerates. blend provides a compromise.
				interpolator_ = &blend;

			CASPAR_LOG(warning) << source_->print() << L" Frame blending frame rate conversion required to conform to channel frame rate.";
		}

		// Note: Uses 1 step rotated cadence for 1001 modes (1602, 1602, 1601, 1602, 1601)
		// This cadence fills the audio mixer most optimally.
		boost::range::rotate(destination_audio_cadence_, std::end(destination_audio_cadence_) - 1);
	}

	draw_frame receive_impl() override
	{
		if (destination_fieldmode_ == field_mode::progressive)
		{
			return do_render_progressive_frame(true);
		}
		else
		{
			auto field1 = do_render_progressive_frame(true);
			auto field2 = do_render_progressive_frame(false);

			return draw_frame::interlace(field1, field2, destination_fieldmode_);
		}
	}

	std::future<std::wstring> call(const std::vector<std::wstring>& params) override
	{
		if (!boost::iequals(params.at(0), L"framerate") || params.size() != 3)
			return source_->call(params);

		if (boost::iequals(params.at(1), L"interpolation"))
		{
			if (boost::iequals(params.at(2), L"blend"))
				interpolator_ = &blend;
			else if (boost::iequals(params.at(2), L"blend_all"))
				interpolator_ = blend_all();
			else
				interpolator_ = &drop_and_skip;
		}
		else if (boost::iequals(params.at(1), L"output_repeat")) // Only for debugging purposes
		{
			output_repeat_ = boost::lexical_cast<unsigned int>(params.at(2));
		}

		return make_ready_future<std::wstring>(L"");
	}

	monitor::subject& monitor_output() override
	{
		return source_->monitor_output();
	}

	std::wstring print() const override
	{
		return source_->print();
	}

	std::wstring name() const override
	{
		return source_->name();
	}

	boost::property_tree::wptree info() const override
	{
		return source_->info();
	}

	constraints& pixel_constraints() override
	{
		return source_->pixel_constraints();
	}
private:
	draw_frame do_render_progressive_frame(bool sound)
	{
		if (output_repeat_ && ++output_frame_ % output_repeat_)
		{
			auto frame = draw_frame::still(last_frame());

			frame.transform().audio_transform.volume = 0.0;

			return attach_sound(frame);
		}

		if (previous_frame_ == draw_frame::empty())
			previous_frame_ = pop_frame_from_source();

		auto current_frame_number	= current_frame_number_;
		auto distance				= current_frame_number_ - boost::rational_cast<int64_t>(current_frame_number_);
		bool needs_next				= distance > 0 || !enough_sound();

		if (needs_next && next_frame_ == draw_frame::empty())
			next_frame_ = pop_frame_from_source();

		auto result = interpolator_(previous_frame_, next_frame_, distance);

		auto next_frame_number		= current_frame_number_ += get_speed();
		auto integer_current_frame	= boost::rational_cast<std::int64_t>(current_frame_number);
		auto integer_next_frame		= boost::rational_cast<std::int64_t>(next_frame_number);

		fast_forward_integer_frames(integer_next_frame - integer_current_frame);

		if (sound)
			return attach_sound(result);
		else
			return result;
	}

	void fast_forward_integer_frames(std::int64_t num_frames)
	{
		if (num_frames == 0)
			return;

		for (std::int64_t i = 0; i < num_frames; ++i)
		{
			previous_frame_ = std::move(next_frame_);

			next_frame_ = pop_frame_from_source();
		}
	}

	boost::rational<std::int64_t> get_speed() const
	{
		return speed_;
	}

	draw_frame pop_frame_from_source()
	{
		auto frame = source_->receive();

		audio_extractor extractor([this](const const_frame& frame)
		{
			if (source_channel_layout_ != frame.audio_channel_layout())
			{
				source_channel_layout_ = frame.audio_channel_layout();

				// Insert silence samples so that the audio mixer is guaranteed to be filled.
				auto min_num_samples_per_frame	= *boost::min_element(destination_audio_cadence_);
				auto max_num_samples_per_frame	= *boost::max_element(destination_audio_cadence_);
				auto cadence_safety_samples		= max_num_samples_per_frame - min_num_samples_per_frame;
				audio_samples_.resize(source_channel_layout_.num_channels * cadence_safety_samples, 0);
			}

			auto& buffer = frame.audio_data();
			audio_samples_.insert(audio_samples_.end(), buffer.begin(), buffer.end());
		});

		frame.accept(extractor);
		frame.transform().audio_transform.volume = 0.0;

		return frame;
	}

	draw_frame attach_sound(draw_frame frame)
	{
		if (source_channel_layout_ == audio_channel_layout::invalid())
			return frame;

		mutable_audio_buffer buffer;

		if (destination_audio_cadence_.front() * source_channel_layout_.num_channels == audio_samples_.size())
		{
			buffer.swap(audio_samples_);
		}
		else if (audio_samples_.size() >= destination_audio_cadence_.front() * source_channel_layout_.num_channels)
		{
			auto begin	= audio_samples_.begin();
			auto end	= begin + destination_audio_cadence_.front() * source_channel_layout_.num_channels;

			buffer.insert(buffer.begin(), begin, end);
			audio_samples_.erase(begin, end);
		}
		else
		{
			auto needed = destination_audio_cadence_.front();
			auto got = audio_samples_.size() / source_channel_layout_.num_channels;
			CASPAR_LOG(debug) << print() << L" Too few audio samples. Needed " << needed << L" but got " << got;
			buffer.swap(audio_samples_);
			buffer.resize(needed * source_channel_layout_.num_channels, 0);
		}

		boost::range::rotate(destination_audio_cadence_, std::begin(destination_audio_cadence_) + 1);

		auto audio_frame = mutable_frame(
				{},
				std::move(buffer),
				this,
				pixel_format_desc(),
				source_channel_layout_);
		return draw_frame::over(frame, draw_frame(std::move(audio_frame)));
	}

	bool enough_sound() const
	{
		return source_channel_layout_ == core::audio_channel_layout::invalid()
				|| audio_samples_.size() / source_channel_layout_.num_channels >= destination_audio_cadence_.at(0);
	}
};

spl::shared_ptr<frame_producer> create_framerate_producer(
		spl::shared_ptr<frame_producer> source,
		boost::rational<int> source_framerate,
		boost::rational<int> destination_framerate,
		field_mode destination_fieldmode,
		std::vector<int> destination_audio_cadence)
{
	return spl::make_shared<framerate_producer>(
			std::move(source),
			std::move(source_framerate),
			std::move(destination_framerate),
			destination_fieldmode,
			std::move(destination_audio_cadence));
}

}}
