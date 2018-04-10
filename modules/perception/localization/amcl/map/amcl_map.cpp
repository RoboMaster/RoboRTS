/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "modules/perception/localization/amcl/map/amcl_map.h"

namespace rrts {
namespace perception {
namespace localization {

AmclMap::~AmclMap() {
	for (int i = 0; i < cells_vec_.size(); i++) {
		delete cells_vec_[i];
	}
	cells_vec_.clear();
	cells_vec_.shrink_to_fit();
}

int AmclMap::GetSizeX() const {
	return size_x_;
}

int AmclMap::GetSizeY() const {
	return size_y_;
}

int AmclMap::ComputeCellIndexByMap(const int &i, const int &j) {
	return i + j * this->size_x_;
};

void AmclMap::ConvertWorldCoordsToMapCoords(const double &x, const double &y,
											int &mx, int &my) {
	mx = (std::floor((x - this->origin_x_) / this->scale_ + 0.5) + this->size_x_ / 2);
	my = (std::floor((y - this->origin_y_) / this->scale_ + 0.5) + this->size_y_ / 2);
}

void AmclMap::ConvertMapCoordsToWorldCoords(const int &x, const int &y,
											double &wx, double &wy) {
	wx = (this->origin_x_ + ((x) - this->size_x_ / 2) * this->scale_);
	wy = (this->origin_y_ + ((y) - this->size_y_ / 2) * this->scale_);
}

bool AmclMap::CheckMapCoordsValid(const int &i, const int &j) {
	return ((i >= 0) && (i < this->size_x_) && (j >= 0) && (j < this->size_y_));
};

void AmclMap::ConvertFromMsg(const nav_msgs::OccupancyGrid &map_msg) {
	this->size_x_ = map_msg.info.width;
	this->size_y_ = map_msg.info.height;
	this->scale_ = map_msg.info.resolution;
	this->origin_x_ = map_msg.info.origin.position.x + (this->size_x_ / 2) * this->scale_;
	this->origin_y_ = map_msg.info.origin.position.y + (this->size_y_ / 2) * this->scale_;
	this->cells_vec_.resize(this->size_x_ * this->size_y_);

	for (int i = 0; i < this->size_x_ * this->size_y_; i++) {
		this->cells_vec_[i] = new Cell();
		auto tmp_msg = static_cast<int>(map_msg.data[i]);
		if (tmp_msg == 0) {
			this->cells_vec_[i]->occ_state = -1;
		} else if (tmp_msg == 100) {
			this->cells_vec_[i]->occ_state = +1;
		} else {
			this->cells_vec_[i]->occ_state = 0;
		}
	}
}

void AmclMap::UpdateCSpace(double max_occ_dist) {

	auto marked = new unsigned char[this->size_x_ * this->size_y_]();
	std::priority_queue<CellData, std::vector<CellData>, Comp> Q;

	this->max_occ_dist_ = max_occ_dist;

	auto cdm = BuildDistanceMap(this->scale_, this->max_occ_dist_);

	// Enqueue all the obstacle cells
	CellData cell;
	cell.map_ptr_ = shared_from_this();
	for (int i = 0; i < this->size_x_; i++) {
		cell.src_i_ = cell.i_ = i;
		for (int j = 0; j < size_y_; j++) {
			auto map_index_tmp = ComputeCellIndexByMap(i, j);
			if (this->cells_vec_[map_index_tmp]->occ_state == +1) {
				this->cells_vec_[map_index_tmp]->occ_dist = 0.0;
				cell.src_j_ = cell.j_ = j;
				marked[map_index_tmp] = 1;
				Q.push(cell);
			} else {
				this->cells_vec_[map_index_tmp]->occ_dist = max_occ_dist;
			}
		}
	}

	while (!Q.empty()) {
		CellData current_cell_data = Q.top();
		if (current_cell_data.i_ > 0) {
			Enqueue(current_cell_data.i_ - 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q, cdm, marked);
		}
		if (current_cell_data.j_ > 0) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ - 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q, cdm, marked);
		}
		if (current_cell_data.i_ < this->size_x_ - 1) {
			Enqueue(current_cell_data.i_ + 1, current_cell_data.j_,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q, cdm, marked);
		}
		if (current_cell_data.j_ < this->size_y_ - 1) {
			Enqueue(current_cell_data.i_, current_cell_data.j_ + 1,
					current_cell_data.src_i_, current_cell_data.src_j_,
					Q, cdm, marked);
		}
		Q.pop();
	}

	delete[] marked;
	delete cdm;

}

void AmclMap::Enqueue(int i, int j, int src_i, int src_j, CellDataPriorityQueue &Q,
					  CachedDistanceMap *cdm_ptr,
					  unsigned char *marked) {

	auto index = ComputeCellIndexByMap(i, j);
	if (marked[index]) {
		return;
	}

	int di = std::abs(i - src_i);
	int dj = std::abs(j - src_j);
	double distance = cdm_ptr->distances_mat_(di, dj);

	if (distance > cdm_ptr->cell_radius_) {
		return;
	}

	this->cells_vec_[index]->occ_dist = distance * this->scale_;

	CellData cell_data(shared_from_this(),
					   static_cast<unsigned int>(i),
					   static_cast<unsigned int>(j),
					   static_cast<unsigned int>(src_i),
					   static_cast<unsigned int>(src_j));

	Q.push(cell_data);
	marked[index] = 1;

}

bool AmclMap::CheckIndexFree(int i, int j) {
	if (this->cells_vec_[ComputeCellIndexByMap(i, j)]->occ_state == -1) {
		return true;
	} else {
		return false;
	}
}

CachedDistanceMap *AmclMap::BuildDistanceMap(double scale, double max_dist) {
	static CachedDistanceMap *cdm = nullptr;
	if (!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist)) {
		if (cdm != nullptr)
			delete cdm;
		cdm = new CachedDistanceMap(scale, max_dist);
	}
	return cdm;
}

const std::vector<Cell *> &AmclMap::GetCellsVec() const {
	return cells_vec_;
}

double AmclMap::GetMaxOccDist() const {
	return max_occ_dist_;
}

CachedDistanceMap::CachedDistanceMap(double scale, double max_dist) : scale_(scale),
																	  max_dist_(max_dist) {
	cell_radius_ = static_cast<int>(max_dist / scale);
	distances_mat_.setZero(cell_radius_ + 2, cell_radius_ + 2);
	//TODO Optimize
	for (int i = 0; i < cell_radius_ + 2; i++) {
		for (int j = 0; j < cell_radius_ + 2; j++) {
			distances_mat_(i, j) = std::sqrt(i * i + j * j);
		}
	}

}

}
}
}

