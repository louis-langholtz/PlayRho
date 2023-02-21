# Benchmark Console Application

This application component provides benchmark data for various functions and methods of PlayRho.
It uses Google's [**benchmark**](https://github.com/google/benchmark) micro-benchmark support library.

## Prerequisites

This application currently relies on the `Benchmark/googlebench` git sub module,
so it has no prerequisites other than those for the [library component](../PlayRho/).

## Configuration

This component needs the `PLAYRHO_BUILD_BENCHMARK` CMake option turned on.
It also makes sense to set the `CMAKE_BUILD_TYPE` option to `Release`.
This can be achieved through the command line interface by adding the
`-DPLAYRHO_BUILD_BENCHMARK=ON` and `-DCMAKE_BUILD_TYPE=Release` arguments to
the CMake configuration step's list of arguments.

## Building & Installation

See the project's [documented build and installation steps](../INSTALL.md) and be sure to add the above mentioned command line arguments.

## Sample Output

Note that the following times are for running the named benchmarks which may have way more overhead than their names suggests.
Don't put much weight into these results unless you're clear on the code that's being timed.

```sh
$ ./Benchmark
Run on (8 X 2600 MHz CPU s)
2017-11-20 18:31:48
-------------------------------------------------------------------------
Benchmark                                  Time           CPU Iterations
-------------------------------------------------------------------------
FloatAdd/1000                            581 ns        579 ns    1174911
FloatMul/1000                            574 ns        573 ns    1217158
FloatDiv/1000                           1995 ns       1994 ns     342374
FloatSqrt/1000                          2045 ns       2045 ns     332186
FloatSin/1000                           6805 ns       6798 ns     101185
FloatCos/1000                           7144 ns       7134 ns      95021
FloatSinCos/1000                        6914 ns       6905 ns      93443
FloatAtan2/1000                         6982 ns       6977 ns      97983
FloatHypot/1000                         4015 ns       4015 ns     174633
DoubleAdd/1000                           585 ns        585 ns    1173256
DoubleMul/1000                           589 ns        588 ns    1121184
DoubleDiv/1000                          4041 ns       4040 ns     171622
DoubleSqrt/1000                         3996 ns       3992 ns     174198
DoubleSin/1000                         11520 ns      11518 ns      59252
DoubleCos/1000                         12289 ns      12281 ns      56609
DoubleSinCos/1000                      13203 ns      13199 ns      52519
DoubleAtan2/1000                       25459 ns      25455 ns      27246
DoubleHypot/1000                        4911 ns       4907 ns     138726
AlmostEqual1/1000                       3177 ns       3176 ns     218750
AlmostEqual2/1000                       1244 ns       1244 ns     520477
DiffSignsViaSignbit/1000                 897 ns        896 ns     773600
DiffSignsViaMul/1000                     869 ns        868 ns     787747
ModuloViaTrunc/1000                     1990 ns       1990 ns     340677
ModuloViaFmod/1000                      6586 ns       6581 ns     105415
DotProduct/1000                          897 ns        896 ns     753596
CrossProduct/1000                        752 ns        751 ns     920859
LengthSquaredViaDotProduct/1000          868 ns        868 ns     796251
GetLengthSquared/1000                    604 ns        603 ns    1076509
GetLength/1000                          1988 ns       1988 ns     333072
UnitVectorFromVector/1000               4480 ns       4476 ns     136012
UnitVectorFromVectorAndBack/1000        4465 ns       4464 ns     153616
UnitVecFromAngle/1000                   7142 ns       7134 ns      97723
AABB2D/1000                             7572 ns       7566 ns      82166
ConstructAndAssignVC                      26 ns         26 ns   26404333
SolveVC                                   43 ns         43 ns   16475007
MaxSepBetweenAbsRectangles                77 ns         77 ns    9091854
MaxSepBetweenRel4x4                       85 ns         85 ns    7717836
MaxSepBetweenRel2_4x4                     86 ns         86 ns    7984214
MaxSepBetweenRelRectanglesNoStop          97 ns         96 ns    7213520
MaxSepBetweenRelRectangles2NoStop         95 ns         95 ns    7001750
MaxSepBetweenRelRectangles               100 ns         99 ns    6777495
MaxSepBetweenRelRectangles2              100 ns         99 ns    6898115
ManifoldForTwoSquares1                   143 ns        143 ns    4644158
ManifoldForTwoSquares2                   138 ns        138 ns    4873532
AsyncFutureDeferred                      281 ns        281 ns    2465752
AsyncFutureAsync                       21436 ns      19139 ns      35243
ThreadCreateAndDestroy                 24129 ns      18756 ns      37133
MultiThreadQD                          10471 ns       5552 ns     100000
MultiThreadQDE                         10048 ns       5336 ns     100000
MultiThreadQDA                           154 ns        154 ns    4572832
MultiThreadQDAQ                       248591 ns     248497 ns      13713
WorldStep                                 75 ns         75 ns    9143633
WorldStepWithStatsStatic/0                87 ns         87 ns    7893105
WorldStepWithStatsStatic/1                93 ns         92 ns    7351088
WorldStepWithStatsStatic/10              128 ns        128 ns    5280986
WorldStepWithStatsStatic/100             362 ns        362 ns    1913991
WorldStepWithStatsStatic/1000           5592 ns       5587 ns     120688
WorldStepWithStatsStatic/10000         91228 ns      91141 ns       7512
DropDisks/0                               76 ns         76 ns    9250449
DropDisks/1                              941 ns        940 ns     739528
DropDisks/10                            9395 ns       9388 ns      70862
DropDisks/100                         100451 ns     100382 ns       6838
DropDisks/1000                        990632 ns     989579 ns        705
DropDisks/10000                     10088055 ns   10079884 ns         69
TumblerAdd100SquaresPlus100Steps    42491661 ns   42488688 ns         16
TumblerAdd200SquaresPlus200Steps   192134232 ns  192130667 ns          3
AddPairStressTest400/0               2906973 ns    2906795 ns        234
AddPairStressTest400/10              1298488 ns    1298009 ns        546
AddPairStressTest400/15              1440968 ns    1440525 ns        493
AddPairStressTest400/16             21259641 ns   21253969 ns         32
AddPairStressTest400/17             48492777 ns   48438417 ns         12
AddPairStressTest400/18             84638874 ns   84596571 ns          7
AddPairStressTest400/19             25832690 ns   25819296 ns         27
AddPairStressTest400/20             21288679 ns   21287455 ns         33
AddPairStressTest400/30              7102015 ns    7099620 ns        100
TilesComesToRest/12                 60359953 ns   60309250 ns         12
TilesComesToRest/20                359062532 ns  358925000 ns          2
TilesComesToRest/36               4043229240 ns 4042139000 ns          1
```
