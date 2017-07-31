# Benchmark Console Application

Welcome to the Benchmark console application.

This application provides benchmark data for various functions and methods of PlayRho. It uses Google's [**benchmark**](https://github.com/google/benchmark) micro-benchmark support library.

## Example Output

```sh
Run on (8 X 2600 MHz CPU s)
2017-07-30 23:47:09
------------------------------------------------------------------
Benchmark                           Time           CPU Iterations
------------------------------------------------------------------
TwoRandValues                      13 ns         13 ns   51558161
FloatAddition                      13 ns         13 ns   51691799
FloatMultiplication                14 ns         14 ns   53026688
FloatDivision                      13 ns         13 ns   53530325
FloatSqrt                          13 ns         13 ns   52333710
FloatSin                           36 ns         36 ns   19653260
FloatCos                           35 ns         35 ns   18938062
DoubleAddition                     14 ns         14 ns   53477979
DoubleMultiplication               14 ns         13 ns   53531962
DoubleDivision                     16 ns         16 ns   43261684
DoubleSqrt                         16 ns         16 ns   43744259
DoubleSin                          43 ns         43 ns   16272069
DoubleCos                          43 ns         43 ns   15854609
LengthSquaredViaDotProduct         13 ns         13 ns   53764267
BM_GetLengthSquared                13 ns         13 ns   53913753
BM_GetLength                       13 ns         13 ns   53345933
UnitVectorFromVec2                 17 ns         17 ns   41125910
FourRandValues                     25 ns         25 ns   26734394
DotProduct                         26 ns         26 ns   26706345
CrossProduct                       26 ns         26 ns   27021915
ConstructAndAssignVC               35 ns         35 ns   20114133
SolveVC                            51 ns         50 ns   14164134
GetMaxSeparation                  132 ns        132 ns    5540210
ManifoldForTwoSquares1            200 ns        200 ns    3351271
ManifoldForTwoSquares2            197 ns        197 ns    3574912
TilesComesToRest             58388557 ns   58374455 ns         11
Program ended with exit code: 0
```
