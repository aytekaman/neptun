// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Jeremie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Jeremie Dumas on 09/04/18.
//

#pragma once

#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/spdlog.h>

namespace floatTetWild {

struct Logger
{
    static std::shared_ptr<spdlog::logger> logger_;

    // By default, write to stdout, but don't write to any file
    static void init(bool                      use_cout = true,
                     const spdlog::filename_t& filename = spdlog::filename_t(),
                     bool                      truncate = true);
};

// Retrieve current logger, or create one if not available
inline spdlog::logger& logger()
{
    if (!Logger::logger_) {
        Logger::init();
    }
    return *Logger::logger_;
}

}  // namespace floatTetWild
