#pragma once

#include <common/forward.h>
#include <common/memory.h>
#include <common/diagnostics/graph.h>

#include <core/fwd.h>

#include <boost/noncopyable.hpp>

namespace caspar { namespace accelerator {

class accelerator : boost::noncopyable
{
public:
	accelerator(const std::wstring& path);
	~accelerator();

	std::unique_ptr<core::image_mixer> create_image_mixer(const spl::shared_ptr<diagnostics::graph>& graph, int channel_id);

	std::shared_ptr<ogl::device> get_ogl_device() const;
private:
	struct impl;
	spl::unique_ptr<impl> impl_;
};

}}
