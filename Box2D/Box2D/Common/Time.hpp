//
//  Timespan.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 11/19/16.
//
//

#ifndef Timespan_hpp
#define Timespan_hpp

#include <Box2D/Common/Settings.hpp>
#include <chrono>

namespace box2d
{

using Time = std::chrono::duration<float_t>;

constexpr Time operator""_s(unsigned long long value)
{
	return Time(value);
}

constexpr Time operator""_s(long double value)
{
	return Time(value);
}
	
} // namespace box2d

#endif /* Timespan_hpp */
