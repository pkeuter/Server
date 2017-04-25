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
* Author: Robert Nagy, ronag89@gmail.com
*/

#pragma once

#include <common/tweener.h>
#include <common/env.h>

#include <core/video_format.h>
#include <core/mixer/image/blend_modes.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_vector.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>

namespace caspar { namespace core {

struct chroma
{
	enum class legacy_type
	{
		none,
		green,
		blue
	};

	bool		enable						= false;
	bool		show_mask					= false;
	double		target_hue					= 0.0;
	double		hue_width					= 0.0;
	double		min_saturation				= 0.0;
	double		min_brightness				= 0.0;
	double		softness					= 0.0;
	double		spill_suppress				= 0.0;
	double		spill_suppress_saturation	= 1.0;
};

struct levels final
{
	double min_input	= 0.0;
	double max_input	= 1.0;
	double gamma		= 1.0;
	double min_output	= 0.0;
	double max_output	= 1.0;
};

struct corners final
{
	boost::array<double, 2> ul = boost::array<double, 2> { { 0.0, 0.0 } };
	boost::array<double, 2> ur = boost::array<double, 2> { { 1.0, 0.0 } };
	boost::array<double, 2> lr = boost::array<double, 2> { { 1.0, 1.0 } };
	boost::array<double, 2> ll = boost::array<double, 2> { { 0.0, 1.0 } };
};

struct rectangle final
{
	boost::array<double, 2> ul = boost::array<double, 2> { { 0.0, 0.0 } };
	boost::array<double, 2> lr = boost::array<double, 2> { { 1.0, 1.0 } };
};

typedef boost::numeric::ublas::matrix<double, boost::numeric::ublas::row_major, std::vector<double>> t_matrix;

struct image_transform final
{
	double					opacity				= 1.0;
	double					contrast			= 1.0;
	double					brightness			= 1.0;
	double					saturation			= 1.0;

	// A bug in VS 2013 prevents us from writing:
	// boost::array<double, 2> fill_translation = { { 0.0, 0.0 } };
	// See http://blogs.msdn.com/b/vcblog/archive/2014/08/19/the-future-of-non-static-data-member-initialization.aspx
	boost::array<double, 2>	anchor				= boost::array<double, 2> { { 0.0, 0.0 } };
	boost::array<double, 2>	fill_translation	= boost::array<double, 2> { { 0.0, 0.0 } };
	boost::array<double, 2>	fill_scale			= boost::array<double, 2> { { 1.0, 1.0 } };
	boost::array<double, 2>	clip_translation	= boost::array<double, 2> { { 0.0, 0.0 } };
	boost::array<double, 2>	clip_scale			= boost::array<double, 2> { { 1.0, 1.0 } };
	double					angle				= 0.0;
	rectangle				crop;
	corners					perspective;
	core::levels			levels;
	core::chroma			chroma;

	core::field_mode		field_mode			= core::field_mode::progressive;
	bool					is_key				= false;
	bool					is_mix				= false;
	bool					use_mipmap			= false;
	core::blend_mode		blend_mode			= core::blend_mode::normal;
	int						layer_depth			= 0;
	//double					aspect_ratio		= 1.0;

	image_transform& operator*=(const image_transform &other);
	image_transform operator*(const image_transform &other) const;

	t_matrix get_transform_matrix() const;

	static image_transform tween(double time, const image_transform& source, const image_transform& dest, double duration, const tweener& tween);
private:
	t_matrix transform_matrix = boost::numeric::ublas::identity_matrix<double>(3, 3);
};

bool operator==(const image_transform& lhs, const image_transform& rhs);
bool operator!=(const image_transform& lhs, const image_transform& rhs);

struct audio_transform final
{
	double	volume		= 1.0;
	bool	is_still	= false;

	audio_transform& operator*=(const audio_transform &other);
	audio_transform operator*(const audio_transform &other) const;

	static audio_transform tween(double time, const audio_transform& source, const audio_transform& dest, double duration, const tweener& tween);
};

bool operator==(const audio_transform& lhs, const audio_transform& rhs);
bool operator!=(const audio_transform& lhs, const audio_transform& rhs);

//__declspec(align(16))
struct frame_transform final
{
public:
	frame_transform();

	core::image_transform image_transform;
	core::audio_transform audio_transform;

	//char padding[(sizeof(core::image_transform) + sizeof(core::audio_transform)) % 16];

	frame_transform& operator*=(const frame_transform &other);
	frame_transform operator*(const frame_transform &other) const;

	static frame_transform tween(double time, const frame_transform& source, const frame_transform& dest, double duration, const tweener& tween);
};

bool operator==(const frame_transform& lhs, const frame_transform& rhs);
bool operator!=(const frame_transform& lhs, const frame_transform& rhs);

t_matrix get_transform_matrix(const image_transform& transform);

t_matrix create_matrix(std::vector<std::vector<double>> data);

template<typename RandomAccessRange>
boost::numeric::ublas::vector<double, std::vector<double>> create_vector(RandomAccessRange array)
{
	boost::numeric::ublas::vector<double, std::vector<double>> vector(3);

	for (int i = 0; i < 3; ++i)
		vector(i) = array.size() > i ? array.at(i) : 1.0;

	return vector;
}

class tweened_transform
{
	frame_transform source_;
	frame_transform dest_;
	int duration_;
	int time_;
	tweener tweener_;
public:
	tweened_transform()
		: duration_(0)
		, time_(0)
	{
		dest_.image_transform.use_mipmap = env::properties().get(L"configuration.mixer.mipmapping-default-on", false);
	}

	tweened_transform(const frame_transform& source, const frame_transform& dest, int duration, const tweener& tween)
		: source_(source)
		, dest_(dest)
		, duration_(duration)
		, time_(0)
		, tweener_(tween)
	{
	}

	const frame_transform& dest() const
	{
		return dest_;
	}

	frame_transform fetch()
	{
		return time_ == duration_ ? dest_ : frame_transform::tween(static_cast<double>(time_), source_, dest_, static_cast<double>(duration_), tweener_);
	}

	frame_transform fetch_and_tick(int num)
	{
		time_ = std::min(time_+num, duration_);
		return fetch();
	}
};

boost::optional<chroma::legacy_type> get_chroma_mode(const std::wstring& str);

namespace detail {

void set_current_aspect_ratio(double aspect_ratio);
double get_current_aspect_ratio();

}}}

namespace boost { namespace numeric { namespace ublas {

template<typename T, typename L, typename S>
boost::numeric::ublas::matrix<T, L, S> operator*(const boost::numeric::ublas::matrix<T, L, S>& lhs, const boost::numeric::ublas::matrix<T, L, S>& rhs)
{
	return boost::numeric::ublas::matrix<T>(boost::numeric::ublas::prod(lhs, rhs));
}

template<typename T, typename L, typename S1, typename S2>
boost::numeric::ublas::vector<T, S1> operator*(const boost::numeric::ublas::vector<T, S1>& lhs, const boost::numeric::ublas::matrix<T, L, S2>& rhs)
{
	return boost::numeric::ublas::vector<T, S1>(boost::numeric::ublas::prod(lhs, rhs));
}

template<typename T, typename L, typename S1, typename S2>
bool operator==(const boost::numeric::ublas::matrix<T, L, S1>& lhs, const boost::numeric::ublas::matrix<T, L, S2>& rhs)
{
	if (lhs.size1() != rhs.size1() || lhs.size2() != rhs.size2())
		return false;

	for (int y = 0; y < lhs.size1(); ++y)
		for (int x = 0; x < lhs.size2(); ++x)
			if (lhs(y, x) != rhs(y, x))
				return false;

	return true;
}

}}}
