/*
 * traversability.h
 *
 *  Modified by: Haoran Wang
 *  Revision date: 2026-08-12
 */

#pragma once

namespace lesta_types {

enum class Traversability : int {
  TRAVERSABLE = 1,
  NON_TRAVERSABLE = 0,
  UNKNOWN = -1,
};
} // namespace lesta_types