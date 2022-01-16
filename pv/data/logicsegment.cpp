/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h" // For HAVE_UNALIGNED_LITTLE_ENDIAN_ACCESS

#include <extdef.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "logic.hpp"
#include "logicsegment.hpp"

#include <libsigrokcxx/libsigrokcxx.hpp>

using std::lock_guard;
using std::recursive_mutex;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;

using sigrok::Logic;

namespace pv {
namespace data {

const int LogicSegment::MipMapScalePower = 4;
const int LogicSegment::MipMapScaleFactor = 1 << MipMapScalePower;
const float LogicSegment::LogMipMapScaleFactor = logf(MipMapScaleFactor);
const uint64_t LogicSegment::MipMapDataUnit = 64 * 1024; // bytes

LogicSegment::LogicSegment(pv::data::Logic& owner, uint32_t segment_id,
	unsigned int unit_size,	uint64_t samplerate) :
	Segment(segment_id, samplerate, unit_size),
	owner_(owner),
	last_append_sample_(0),
	last_append_accumulator_(0),
	last_append_extra_(0)
{
}

LogicSegment::~LogicSegment()
{
}

shared_ptr<const LogicSegment> LogicSegment::get_shared_ptr() const
{
	shared_ptr<const Segment> ptr = nullptr;

	try {
		ptr = shared_from_this();
	} catch (std::exception& e) {
		/* Do nothing, ptr remains a null pointer */
	}

	return ptr ? std::dynamic_pointer_cast<const LogicSegment>(ptr) : nullptr;
}

void LogicSegment::append_payload(shared_ptr<sigrok::Logic> logic)
{
	assert(unit_size_ == logic->unit_size());
	assert((logic->data_length() % unit_size_) == 0);

	append_payload(logic->data_pointer(), logic->data_length());
}

void LogicSegment::append_payload(void *data, uint64_t data_size)
{
	assert(unit_size_ > 0);
	assert((data_size % unit_size_) == 0);

	lock_guard<recursive_mutex> lock(mutex_);

	const uint64_t prev_sample_count = sample_count_;
	const uint64_t sample_count = data_size / unit_size_;

	append_samples(data, sample_count);


	if (sample_count > 1)
		owner_.notify_samples_added(SharedPtrToSegment(shared_from_this()),
			prev_sample_count + 1, prev_sample_count + 1 + sample_count);
	else
		owner_.notify_samples_added(SharedPtrToSegment(shared_from_this()),
			prev_sample_count + 1, prev_sample_count + 1);
}

void LogicSegment::get_samples(int64_t start_sample,
	int64_t end_sample, uint8_t* dest) const
{
	assert(start_sample >= 0);
	assert(start_sample <= (int64_t)sample_count_);
	assert(end_sample >= 0);
	assert(end_sample <= (int64_t)sample_count_);
	assert(start_sample <= end_sample);
	assert(dest != nullptr);

	lock_guard<recursive_mutex> lock(mutex_);

	get_raw_samples(start_sample, (end_sample - start_sample), dest);
}

void LogicSegment::get_subsampled_edges(
	vector<EdgePair> &edges,
	uint64_t start, uint64_t end,
	float min_length, int sig_index, bool first_change_only)
{
	RLESample rle_sample;
	uint64_t last_captured_edge_index;
	uint64_t ff_start_index;
	bool last_sample;
	bool sample;
	bool fast_forward = false;

	assert(start <= end);
	assert(min_length > 0);
	assert(sig_index >= 0);
	assert(sig_index < 64);

	lock_guard<recursive_mutex> lock(mutex_);

	// Make sure we only process as many samples as we have
	if (end > get_sample_count())
		end = get_sample_count();

	const uint64_t block_length = (uint64_t)max(min_length, 1.0f);
	const uint64_t sig_mask = 1ULL << sig_index;

	const uint64_t count = rle_sample_count_;
	const uint64_t rle_start_index = start > 0 ? search_for_rle_index(start, 0, count - 1) : 0;
	
	rle_sample = rle_samples_.at(rle_start_index);
	sample = (rle_sample.value & sig_mask) != 0;
	if (!first_change_only)
		edges.emplace_back(start, sample);
	last_sample = sample;
	last_captured_edge_index = start;

	for (uint64_t i = rle_start_index + 1; i < count; i++) {
		rle_sample = rle_samples_.at(i);
		if (rle_sample.sample_index > end)
			break;
		sample = (rle_sample.value & sig_mask) != 0;

		if ((last_sample != sample) || fast_forward) {
			/* if the next edge would be to close to the last one, it would make no sense
			to draw it, so just fast-forward */ 
			if ((rle_sample.sample_index - last_captured_edge_index) < block_length) {
				const uint64_t next_index = min(end, last_captured_edge_index + block_length);
				ff_start_index = last_captured_edge_index;
				i = search_for_rle_index(next_index, i, count-1);
				rle_sample = rle_samples_.at(i);
				sample = (rle_sample.value & sig_mask) != 0;

				uint64_t search_back_index = 0;
				do 
				{
					RLESample last_rle_sample_before_ff = rle_samples_.at(i - search_back_index++);
					const bool last_sample_before_ff = (last_rle_sample_before_ff.value & sig_mask) != 0;
					if (ff_start_index == last_rle_sample_before_ff.sample_index)
						break;
					if (sample != last_sample_before_ff) {
						edges.emplace_back(last_rle_sample_before_ff.sample_index, last_sample_before_ff);
						break;
					}
				} while (true);
				if (rle_sample.sample_index > end) {
					last_sample = sample;
					break;
				}
			}
			fast_forward = false;
			last_captured_edge_index = rle_sample.sample_index;
			edges.emplace_back(rle_sample.sample_index, sample);
			last_sample = sample;
			if (first_change_only)
				return;
		}
	}
	edges.emplace_back(end, last_sample);
}

void LogicSegment::get_surrounding_edges(vector<EdgePair> &dest,
	uint64_t origin_sample, float min_length, int sig_index, uint64_t start_sample)
{
	if (origin_sample >= sample_count_)
		return;
	if (start_sample > origin_sample)
		return;

	// Put the edges vector on the heap, it can become quite big until we can
	// use a get_subsampled_edges() implementation that searches backwards
	vector<EdgePair>* edges = new vector<EdgePair>;

	// Get all edges to the left of origin_sample
	get_subsampled_edges(*edges, start_sample, origin_sample, min_length, sig_index, false);

	// If we don't specify "first only", the first and last edge are the states
	// at samples 0 and origin_sample. If only those exist, there are no edges
	if (edges->size() == 2) {
		delete edges;
		return;
	}

	// Dismiss the entry for origin_sample so that back() gives us the
	// real last entry
	edges->pop_back();
	dest.push_back(edges->back());
	edges->clear();

	// Get first edge to the right of origin_sample
	get_subsampled_edges(*edges, origin_sample, sample_count_, min_length, sig_index, true);

	// "first only" is specified, so nothing needs to be dismissed
	if (edges->size() == 0) {
		delete edges;
		return;
	}

	dest.push_back(edges->front());

	delete edges;
}

uint64_t LogicSegment::pow2_ceil(uint64_t x, unsigned int power)
{
	const uint64_t p = UINT64_C(1) << power;
	return (x + p - 1) / p * p;
}

} // namespace data
} // namespace pv
