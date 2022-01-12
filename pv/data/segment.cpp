/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2017 Soeren Apel <soeren@apelpie.net>
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

#include "segment.hpp"

#include <cassert>
#include <cstdlib>
#include <cstring>

#include <QDebug>

using std::bad_alloc;
using std::lock_guard;
using std::min;
using std::recursive_mutex;

namespace pv {
namespace data {

const uint64_t Segment::MaxChunkSize = 10 * 1024 * 1024;  /* 10MiB */

Segment::Segment(uint32_t segment_id, uint64_t samplerate, unsigned int unit_size) :
	segment_id_(segment_id),
	sample_count_(0),
	rle_sample_count_(0),
	start_time_(0),
	samplerate_(samplerate),
	unit_size_(unit_size),
	iterator_count_(0),
	mem_optimization_requested_(false),
	is_complete_(false)
{
	assert(unit_size_ > 0);

	// Determine the number of samples we can fit in one chunk
	// without exceeding MaxChunkSize
	chunk_size_ = min(MaxChunkSize, (MaxChunkSize / unit_size_) * unit_size_);

	// Create the initial chunk
	current_chunk_ = new uint8_t[chunk_size_ + 7];  /* FIXME +7 is workaround for #1284 */
	data_chunks_.push_back(current_chunk_);
	used_samples_ = 0;
	unused_samples_ = chunk_size_ / unit_size_;
}

Segment::~Segment()
{
	lock_guard<recursive_mutex> lock(mutex_);

	for (uint8_t* chunk : data_chunks_)
		delete[] chunk;
}

uint64_t Segment::get_sample_count() const
{
	return sample_count_;
}

const pv::util::Timestamp& Segment::start_time() const
{
	return start_time_;
}

double Segment::samplerate() const
{
	return samplerate_;
}

void Segment::set_samplerate(double samplerate)
{
	samplerate_ = samplerate;
}

unsigned int Segment::unit_size() const
{
	return unit_size_;
}

uint32_t Segment::segment_id() const
{
	return segment_id_;
}

void Segment::set_complete()
{
	is_complete_ = true;

	completed();
}

bool Segment::is_complete() const
{
	return is_complete_;
}

void Segment::free_unused_memory()
{
	lock_guard<recursive_mutex> lock(mutex_);

	// Do not mess with the data chunks if we have iterators pointing at them
	if (iterator_count_ > 0) {
		mem_optimization_requested_ = true;
		return;
	}

	if (current_chunk_) {
		// No more data will come in, so re-create the last chunk accordingly
		uint8_t* resized_chunk = new uint8_t[used_samples_ * unit_size_ + 7];  /* FIXME +7 is workaround for #1284 */
		memcpy(resized_chunk, current_chunk_, used_samples_ * unit_size_);

		delete[] current_chunk_;
		current_chunk_ = resized_chunk;

		data_chunks_.pop_back();
		data_chunks_.push_back(resized_chunk);
	}
}

void Segment::append_single_sample(void *data)
{
	lock_guard<recursive_mutex> lock(mutex_);

	// There will always be space for at least one sample in
	// the current chunk, so we do not need to test for space

	memcpy(current_chunk_ + (used_samples_ * unit_size_), data, unit_size_);
	used_samples_++;
	unused_samples_--;

	if (unused_samples_ == 0) {
		current_chunk_ = new uint8_t[chunk_size_ + 7];  /* FIXME +7 is workaround for #1284 */
		data_chunks_.push_back(current_chunk_);
		used_samples_ = 0;
		unused_samples_ = chunk_size_ / unit_size_;
	}

	sample_count_++;
}

template <class T>
void Segment::copy_rle_samples(void* data, uint64_t samples)
{
	T *data_ptr = (T *)data;
	uint64_t i = 0;
	RLESample last_sample;

	if (rle_sample_count_ == 0) {
		last_sample.sample_index = 0;
		last_sample.value = *data_ptr;
		rle_samples_.push_back(last_sample);
		rle_sample_count_ = 1;
		last_sample.sample_index++;
		data_ptr++;
		i++;
	} else {
		last_sample = rle_samples_.back();
		rle_samples_.pop_back();
	}

	for (; i < samples; i++) {
		if (last_sample.value != *data_ptr) {
			last_sample.value = *data_ptr;
			rle_samples_.push_back(last_sample);
			rle_sample_count_++;
		}
		last_sample.sample_index++;
		data_ptr++;
	}
	rle_samples_.push_back(last_sample);
}

void Segment::append_samples(void* data, uint64_t samples)
{
	lock_guard<recursive_mutex> lock(mutex_);

	if (unit_size_ == 1)
		copy_rle_samples<uint8_t>(data, samples);
	else if (unit_size_ == 2)
		copy_rle_samples<uint16_t>(data, samples);
	else if (unit_size_ == 4)
		copy_rle_samples<uint32_t>(data, samples);
	else if (unit_size_ == 8)
		copy_rle_samples<uint64_t>(data, samples);
	else
		copy_rle_samples<uint8_t>(data, samples*unit_size_);
	sample_count_ += samples;
}

const uint8_t* Segment::get_raw_sample(uint64_t sample_num) const
{
	assert(sample_num <= sample_count_);

	uint64_t chunk_num = (sample_num * unit_size_) / chunk_size_;
	uint64_t chunk_offs = (sample_num * unit_size_) % chunk_size_;

	lock_guard<recursive_mutex> lock(mutex_);  // Because of free_unused_memory()

	const uint8_t* chunk = data_chunks_[chunk_num];

	return chunk + chunk_offs;
}

uint64_t Segment::search_for_rle_index(uint64_t index, uint64_t search_start, uint64_t search_end) const
{
	uint64_t mid = (search_start + search_end) / 2;
	if ((mid + 1) <= search_end) {
		if (rle_samples_.at(mid).sample_index <= index) {
			if (rle_samples_.at(mid+1).sample_index > index)
				return mid;
			else
				return search_for_rle_index(index, mid+1, search_end);
		}
		else
			return search_for_rle_index(index, search_start, mid);
	}
	return search_end;
}


void Segment::get_raw_samples(uint64_t start, uint64_t count, uint8_t* dest) const
{
	assert(start < sample_count_);
	assert(start + count <= sample_count_);
	assert(count > 0);
	assert(dest != nullptr);

	uint64_t sample_index = start;
	RLESample rle_sample;
	RLESample next_rle_sample;

	assert(unit_size_ <= 8);

	uint64_t rle_index = search_for_rle_index(start, 0, rle_sample_count_ - 1);

	rle_sample = rle_samples_.at(rle_index);
	if ((rle_index + 1) < rle_sample_count_)
		next_rle_sample = rle_samples_.at(rle_index + 1);
	else
		next_rle_sample = rle_sample;
	rle_index++;
	for (uint64_t i = 0; i < count; i++) {

		if (sample_index++ == next_rle_sample.sample_index) {
			rle_sample = next_rle_sample;
			if ((rle_index + 1) < rle_sample_count_) {
				next_rle_sample = rle_samples_.at(rle_index + 1);
			}
			rle_index++;
		}
		switch (unit_size_)
		{
			case sizeof(uint64_t):
				*(uint64_t*)dest = rle_sample.value;
				break;
			case sizeof(uint32_t):
				*(uint32_t*)dest = (uint32_t)rle_sample.value;
				break;
			case sizeof(uint16_t):
				*(uint16_t*)dest = (uint16_t)rle_sample.value;
				break;
			case sizeof(uint8_t):
				*dest = (uint8_t)rle_sample.value;
				break;
			default:
				memcpy(dest, &rle_sample.value, unit_size_);
		}
		dest += unit_size_;
	}
}

SegmentDataIterator* Segment::begin_sample_iteration(uint64_t start)
{
	SegmentDataIterator* it = new SegmentDataIterator;

	assert(start < sample_count_);

	iterator_count_++;

	it->sample_index = start;
	it->chunk_num = (start * unit_size_) / chunk_size_;
	it->chunk_offs = (start * unit_size_) % chunk_size_;
	it->chunk = data_chunks_[it->chunk_num];

	return it;
}

void Segment::continue_sample_iteration(SegmentDataIterator* it, uint64_t increase)
{
	it->sample_index += increase;
	it->chunk_offs += (increase * unit_size_);

	if (it->chunk_offs > (chunk_size_ - 1)) {
		it->chunk_num++;
		it->chunk_offs -= chunk_size_;
		it->chunk = data_chunks_[it->chunk_num];
	}
}

void Segment::end_sample_iteration(SegmentDataIterator* it)
{
	delete it;

	iterator_count_--;

	if ((iterator_count_ == 0) && mem_optimization_requested_) {
		mem_optimization_requested_ = false;
		free_unused_memory();
	}
}

uint8_t* Segment::get_iterator_value(SegmentDataIterator* it)
{
	assert(it->sample_index <= (sample_count_ - 1));

	return (it->chunk + it->chunk_offs);
}

uint64_t Segment::get_iterator_valid_length(SegmentDataIterator* it)
{
	assert(it->sample_index <= (sample_count_ - 1));

	return ((chunk_size_ - it->chunk_offs) / unit_size_);
}

} // namespace data
} // namespace pv
