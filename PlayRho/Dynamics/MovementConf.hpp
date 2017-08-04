/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef MovementConf_hpp
#define MovementConf_hpp

#include <PlayRho/Common/Units.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>

namespace playrho
{
    
    struct MovementConf
    {
        Length maxTranslation;
        Angle maxRotation;
    };
    
    inline MovementConf GetMovementConf(const StepConf& conf)
    {
        return MovementConf{conf.maxTranslation, conf.maxRotation};
    }

} // namespace playrho

#endif /* MovementConf_hpp */
