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

#ifndef COMMON_MACRO_H_
#define COMMON_MACRO_H_

/**
 * @brief Macro definition for class feature including
 *        DISALLOW_COPY, DISALLOW_ASSIGN, DISALLOW_COPY_AND_ASSIGN,
 *        DISALLOW_IMPLICIT_CONSTRUCTORS and DECLARE_SINGLETON
 */
#define DISALLOW_COPY(ClassName)                      \
  public:                                             \
    ClassName(ClassName const&) = delete;             \

#define DISALLOW_ASSIGN(ClassName)                    \
  public:                                             \
    void operator=(ClassName const&) = delete;        \

#define DISALLOW_COPY_AND_ASSIGN(ClassName)           \
  public:                                             \
    ClassName(ClassName const&) = delete;             \
    void operator=(ClassName const&) = delete;        \

#define DISALLOW_IMPLICIT_CONSTRUCTORS(ClassName)     \
  private:                                            \
   ClassName()=default;                               \
  DISALLOW_COPY_AND_ASSIGN(ClassName);                \

#define DECLARE_SINGLETON(ClassName)                  \
 public:                                              \
  static ClassName& GetInstance() {                   \
    static ClassName instance;                        \
    return instance;                                  \
  }                                                   \
  DISALLOW_IMPLICIT_CONSTRUCTORS(ClassName);          \
 private:
#endif  // COMMON_MACRO_H_
