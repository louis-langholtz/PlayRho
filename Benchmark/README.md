# Benchmark Console Application

Welcome to the Benchmark console application.

This application provides benchmark data for various functions and methods of PlayRho. It uses Google's [**benchmark**](https://github.com/google/benchmark) micro-benchmark support library.

## Build Instructions

Coming.

## Sample Output

Note that the following times are for running the named benchmarks which may have way more overhead than their names suggests. What's telling however from this output, is that operations like sine and cosine take significantly longer than square root or division.

```sh
Run on (8 X 2600 MHz CPU s)
2017-07-30 23:55:17
------------------------------------------------------------------
Benchmark                           Time           CPU Iterations
------------------------------------------------------------------
TwoRandValues                      13 ns         13 ns   51577916
FloatAddition                      14 ns         14 ns   51504672
FloatMultiplication                13 ns         13 ns   50828874
FloatDivision                      14 ns         14 ns   51230633
FloatSqrt                          14 ns         14 ns   51709746
FloatSin                           36 ns         36 ns   19214616
FloatCos                           38 ns         38 ns   18015282
DoubleAddition                     14 ns         14 ns   51263649
DoubleMultiplication               14 ns         14 ns   50961349
DoubleDivision                     17 ns         17 ns   41517402
DoubleSqrt                         17 ns         17 ns   41396113
DoubleSin                          47 ns         47 ns   14996936
DoubleCos                          46 ns         46 ns   15167701
LengthSquaredViaDotProduct         14 ns         14 ns   51132587
BM_GetLengthSquared                14 ns         14 ns   51320777
BM_GetLength                       14 ns         14 ns   52137643
UnitVectorFromVec2                 18 ns         18 ns   39641415
FourRandValues                     27 ns         27 ns   26449229
DotProduct                         27 ns         27 ns   26235013
CrossProduct                       27 ns         27 ns   25916999
ConstructAndAssignVC               31 ns         31 ns   22595296
SolveVC                            41 ns         41 ns   16685299
GetMaxSeparation                  104 ns        103 ns    6795655
ManifoldForTwoSquares1            175 ns        175 ns    3920075
ManifoldForTwoSquares2            198 ns        198 ns    3505100
TilesComesToRest             54129850 ns   54124818 ns         11
Program ended with exit code: 0
```
