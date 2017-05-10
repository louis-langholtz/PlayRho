# Testbed GUI Application Tests Directory

This directory is for classes which sub-class the Testbed's `Test` base class.
You don't really need to know anything about it unless you want to add your own
code to the Testbed.

Subclassing `Test` is *part one* of the way to integrate code into the Testbed
GUI application for demoing and visually testing code. After getting the
subclass initially setup, *part two* of integrating code into the Testbed,
is to add a reference to the subclass to the
[TestEntry.cpp](../Framework/TestEntry.cpp) file that's over in the `Framework`
directory.

See also the [Testbed README](../README.md).
