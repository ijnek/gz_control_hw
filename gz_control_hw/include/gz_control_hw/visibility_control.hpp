// Copyright 2022, Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GZ_CONTROL_HW__VISIBILITY_CONTROL_HPP_
#define GZ_CONTROL_HW__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GZ_CONTROL_HW_EXPORT __attribute__ ((dllexport))
    #define GZ_CONTROL_HW_IMPORT __attribute__ ((dllimport))
  #else
    #define GZ_CONTROL_HW_EXPORT __declspec(dllexport)
    #define GZ_CONTROL_HW_IMPORT __declspec(dllimport)
  #endif
  #ifdef GZ_CONTROL_HW_BUILDING_LIBRARY
    #define GZ_CONTROL_HW_PUBLIC GZ_CONTROL_HW_EXPORT
  #else
    #define GZ_CONTROL_HW_PUBLIC GZ_CONTROL_HW_IMPORT
  #endif
  #define GZ_CONTROL_HW_PUBLIC_TYPE GZ_CONTROL_HW_PUBLIC
  #define GZ_CONTROL_HW_LOCAL
#else
  #define GZ_CONTROL_HW_EXPORT __attribute__ ((visibility("default")))
  #define GZ_CONTROL_HW_IMPORT
  #if __GNUC__ >= 4
    #define GZ_CONTROL_HW_PUBLIC __attribute__ ((visibility("default")))
    #define GZ_CONTROL_HW_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GZ_CONTROL_HW_PUBLIC
    #define GZ_CONTROL_HW_LOCAL
  #endif
  #define GZ_CONTROL_HW_PUBLIC_TYPE
#endif

#endif  // GZ_CONTROL_HW__VISIBILITY_CONTROL_HPP_
