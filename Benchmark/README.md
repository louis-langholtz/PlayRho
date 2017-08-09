# Benchmark Console Application

Welcome to the Benchmark console application.

This application provides benchmark data for various functions and methods of PlayRho. It uses Google's [**benchmark**](https://github.com/google/benchmark) micro-benchmark support library.

## Build Instructions

Coming.

## Sample Output

Note that the following times are for running the named benchmarks which may have way more overhead than their names suggests. What's seemingly significant from this output however, is that operations like sine and cosine take significantly longer than square root or division.

```sh
Run on (8 X 2600 MHz CPU s)
2017-08-08 20:51:19
--------------------------------------------------------------------------
Benchmark                                   Time           CPU Iterations
--------------------------------------------------------------------------
FloatAdd                                    2 ns          2 ns  368982294
FloatMult                                   2 ns          2 ns  366233291
FloatDiv                                    2 ns          2 ns  358505544
FloatSqrt                                   2 ns          2 ns  382018915
FloatSin                                    7 ns          7 ns  113156916
FloatCos                                    6 ns          6 ns  110023105
FloatAtan2                                  7 ns          7 ns   99072960
DotProduct                                  2 ns          2 ns  373072680
CrossProduct                                2 ns          2 ns  363119508
LengthSquaredViaDotProduct                  2 ns          2 ns  351751724
GetLengthSquared                            2 ns          2 ns  379617779
GetLength                                   2 ns          2 ns  384497078
hypot                                       4 ns          4 ns  175868792
UnitVectorFromVector                        2 ns          2 ns  294130005
UnitVectorFromVectorAndBack                 2 ns          2 ns  297693724
UnitVecFromAngle                            8 ns          8 ns   80838877
TwoRandValues                              13 ns         13 ns   54730686
FloatAddTwoRand                            13 ns         13 ns   54026102
FloatMultTwoRand                           13 ns         13 ns   53356505
FloatDivTwoRand                            13 ns         13 ns   53571702
FloatSqrtTwoRand                           13 ns         13 ns   53826280
FloatSinTwoRand                            36 ns         36 ns   18716828
FloatCosTwoRand                            35 ns         35 ns   20197240
FloatAtan2TwoRand                          28 ns         28 ns   25564146
DoubleAddTwoRand                           14 ns         14 ns   53771288
DoubleMultTwoRand                          14 ns         14 ns   51642604
DoubleDivTwoRand                           18 ns         17 ns   40536706
DoubleSqrtTwoRand                          18 ns         18 ns   38975718
DoubleSinTwoRand                           49 ns         49 ns   14750226
DoubleCosTwoRand                           48 ns         48 ns   14213313
DoubleAtan2TwoRand                         69 ns         69 ns   10425199
LengthSquaredViaDotProductTwoRand          14 ns         14 ns   50312657
GetLengthSquaredTwoRand                    15 ns         15 ns   48167237
GetLengthTwoRand                           14 ns         14 ns   50834780
hypotTwoRand                               15 ns         15 ns   44679900
UnitVectorFromVectorTwoRand                17 ns         17 ns   43387444
UnitVectorFromVectorAndBackTwoRand         17 ns         17 ns   41706884
FourRandValues                             28 ns         28 ns   24096717
DotProductFourRand                         29 ns         29 ns   24427097
CrossProductFourRand                       29 ns         29 ns   24291133
ConstructAndAssignVC                       33 ns         33 ns   20916126
SolveVC                                    44 ns         44 ns   15413987
MaxSepBetweenAbsRectangles                 74 ns         74 ns    9816020
MaxSepBetweenRel4x4                        80 ns         80 ns    8796069
MaxSepBetweenRel2_4x4                      82 ns         82 ns    8408913
MaxSepBetweenRelRectanglesNoStop          104 ns        104 ns    6918432
MaxSepBetweenRelRectangles2NoStop         103 ns        103 ns    6850119
MaxSepBetweenRelRectangles                104 ns        104 ns    6884275
MaxSepBetweenRelRectangles2               100 ns        100 ns    7195058
ManifoldForTwoSquares1                    156 ns        155 ns    4617475
ManifoldForTwoSquares2                    157 ns        157 ns    4675863
TilesComesToRest12                   57637128 ns   57506769 ns         13
TilesComesToRest20                  375657450 ns  375055500 ns          2
TilesComesToRest36                 3974857636 ns 3973611000 ns          1
Program ended with exit code: 0
```
