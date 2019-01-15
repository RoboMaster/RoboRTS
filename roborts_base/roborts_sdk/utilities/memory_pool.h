/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifndef ROBORTS_SDK_MEMORY_POOL_H
#define ROBORTS_SDK_MEMORY_POOL_H
#include <stdint.h>
#include <cstring>
#include <mutex>
/**
 * @brief Memory block information
 */
typedef struct MemoryBlock
{
  //! flag of usage
  bool usage_flag;
  //! memory block index in the table
  uint8_t table_index;
  //! memory size
  uint16_t memory_size;
  //! memory pointer
  uint8_t* memory_ptr;
} MemoryBlock;

/**
 * @brief Memory pool to manage memory blocks
 */
class MemoryPool {
 public:
  /**
   * @brief Constructor of memory pool
   * @param max_block_size Max size for single memory block
   * @param memory_size Max memory size
   * @param memory_table_num Memory block number in the table
   */
  MemoryPool(uint16_t max_block_size = 1024,
             uint16_t memory_size = 1024,
             uint16_t memory_table_num = 32):
      memory_size_(memory_size),
      memory_table_num_(memory_table_num),
      max_block_size_(max_block_size)
  {
  };
  /**
   * @brief Destructor of memory pool
   */
  ~MemoryPool(){
    delete []memory_;
    delete []memory_table_;
  }
  /**
   * @brief Initialization of memory table
   */
  void Init()
  {
    memory_table_ = new MemoryBlock[memory_table_num_];
    memory_ = new uint8_t[memory_size_];
    uint32_t i;
    memory_table_[0].table_index = 0;
    memory_table_[0].usage_flag  = 1;
    memory_table_[0].memory_ptr  = memory_;
    memory_table_[0].memory_size = 0;
    for (i = 1; i < (memory_table_num_ - 1); i++)
    {
      memory_table_[i].table_index = i;
      memory_table_[i].usage_flag  = 0;
    }
    memory_table_[memory_table_num_ - 1].table_index = memory_table_num_ - 1;
    memory_table_[memory_table_num_ - 1].usage_flag  = 1;
    memory_table_[memory_table_num_ - 1].memory_ptr  = memory_ + memory_size_;
    memory_table_[memory_table_num_ - 1].memory_size = 0;
  }
  /**
   * @brief Free certain memory block in the pool
   * @param memory_block Memory block to be freed
   */
  void FreeMemory(MemoryBlock* memory_block)
  {
    if (memory_block == (MemoryBlock*)0)
    {
      return;
    }
    if (memory_block->table_index == 0 ||
        memory_block->table_index == (memory_table_num_ - 1))
    {
      return;
    }
    memory_block->usage_flag = 0;
  }
  /**
   * @brief Allocate a memory block in the pool
   * @param size size of the memory block
   * @return pointer of memory block
   */
  MemoryBlock* AllocMemory(uint16_t size)
  {
    uint32_t used_memory_size = 0;
    uint8_t  i = 0;
    uint8_t  j = 0;
    uint8_t  memory_table_used_num = 0;
    uint8_t  memory_table_used_index[memory_table_num_];

    uint32_t block_memory_size;
    uint32_t temp_area[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
    uint32_t accumulate_left_memory_size = 0;
    bool index_found = false;

    // If size is larger than memory size or max size for PACKAGE, allocate failed
    if (size>max_block_size_||size > memory_size_)
    {
      return (MemoryBlock *) 0;
    }

    // Calculate the used memory size and get the used index array
    for (i = 0; i < memory_table_num_; i++)
    {
      if (memory_table_[i].usage_flag == 1)
      {
        used_memory_size += memory_table_[i].memory_size;
        memory_table_used_index[memory_table_used_num++] = memory_table_[i].table_index;
      }
    }

    // If left size is smaller than needed, allocate failed
    if (memory_size_ < (used_memory_size + size))
    {
      return (MemoryBlock *) 0;
    }

    // Special case: allocate for the first time
    if (used_memory_size == 0)
    {
      memory_table_[1].memory_ptr      = memory_table_[0].memory_ptr;
      memory_table_[1].memory_size   = size;
      memory_table_[1].usage_flag = 1;
      return &memory_table_[1];
    }

    //memory_table is out of order, memory_table_used_index is ordered by its memory allocation order
    for (i = 0; i < (memory_table_used_num - 1); i++)
    {
      for (j = 0; j < (memory_table_used_num - i - 1); j++)
      {
        if (memory_table_[memory_table_used_index[j]].memory_ptr >
            memory_table_[memory_table_used_index[j + 1]].memory_ptr)
        {
          memory_table_used_index[j + 1] ^= memory_table_used_index[j];
          memory_table_used_index[j] ^= memory_table_used_index[j + 1];
          memory_table_used_index[j + 1] ^= memory_table_used_index[j];
        }
      }
    }

    for (i = 0; i < (memory_table_used_num - 1); i++)
    {
      //Find memory size for each block
      block_memory_size = static_cast<uint32_t>(memory_table_[memory_table_used_index[i + 1]].memory_ptr -
          memory_table_[memory_table_used_index[i]].memory_ptr);

      //Check if block left memory size is enough for needed size, if so, then record block table id and its left memory
      if ((block_memory_size - memory_table_[memory_table_used_index[i]].memory_size) >= size)
      {
        if (temp_area[1] > (block_memory_size - memory_table_[memory_table_used_index[i]].memory_size))
        {
          temp_area[0] = memory_table_[memory_table_used_index[i]].table_index;
          temp_area[1] = block_memory_size - memory_table_[memory_table_used_index[i]].memory_size;
        }
      }

      //accumulate the left memory size for each block
      accumulate_left_memory_size += block_memory_size - memory_table_[memory_table_used_index[i]].memory_size;
      //record the index when accumulate_left_memory_size is larger than needed size for the first time
      if (accumulate_left_memory_size >= size && !index_found)
      {
        j          = i;
        index_found = true;
      }
    }

    //If no single block is available to divide for needed size, then compress the table according to accumulate memory
    if (temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF)
    {
      for (i = 0; i < j; i++)
      {
        if (memory_table_[memory_table_used_index[i + 1]].memory_ptr >
            (memory_table_[memory_table_used_index[i]].memory_ptr +
                memory_table_[memory_table_used_index[i]].memory_size))
        {
          memmove(memory_table_[memory_table_used_index[i]].memory_ptr +
                      memory_table_[memory_table_used_index[i]].memory_size,
                  memory_table_[memory_table_used_index[i + 1]].memory_ptr,
                  memory_table_[memory_table_used_index[i + 1]].memory_size);
          memory_table_[memory_table_used_index[i + 1]].memory_ptr =
              memory_table_[memory_table_used_index[i]].memory_ptr +
                  memory_table_[memory_table_used_index[i]].memory_size;
        }
      }

      for (i = 1; i < (memory_table_num_ - 1); i++)
      {
        if (memory_table_[i].usage_flag == 0)
        {
          memory_table_[i].memory_ptr = memory_table_[memory_table_used_index[j]].memory_ptr +
              memory_table_[memory_table_used_index[j]].memory_size;

          memory_table_[i].memory_size   = size;
          memory_table_[i].usage_flag = 1;
          return &memory_table_[i];
        }
      }
      return (MemoryBlock*)0;
    }

    //If single block is available to divide for needed size, then divide this block into two
    for (i = 1; i < (memory_table_num_ - 1); i++)
    {
      if (memory_table_[i].usage_flag == 0)
      {
        memory_table_[i].memory_ptr =
            memory_table_[temp_area[0]].memory_ptr + memory_table_[temp_area[0]].memory_size;

        memory_table_[i].memory_size   = size;
        memory_table_[i].usage_flag = 1;
        return &memory_table_[i];
      }
    }

    return (MemoryBlock*)0;
  }
  /**
   * @brief Lock the memory pool
   */
  void LockMemory(){
    memory_mutex_.lock();
  }
/**
 * @brief Unlock the memor pool
 */
  void UnlockMemory(){
    memory_mutex_.unlock();
  }

 private:
  //! mutex of the memory pool
  std::mutex memory_mutex_;
  //! number of memory block in the table
  uint16_t memory_table_num_;
  //! max memory size
  uint16_t memory_size_;
  //! max size for single memory block
  uint16_t max_block_size_;
  //! memory table pointer
  MemoryBlock* memory_table_;
  //! memory pointer
  uint8_t* memory_;
};

#endif //ROBORTS_SDK_MEMORY_POOL_H
