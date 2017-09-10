# PlayRho Library Style Guide

This is the style guide for the PlayRho library. This guide is meant for contributors who want to contribute pull requests for code changes.

For general information on contributing, see the [Contributing](../CONTRIBUTING.md) document.

## General Guidelines

### Follow The C++ Core Guidelines For Anything Not Described Herein
For anything not described herein, please follow the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md).

### C++14 Is The Applicable Standard
Write code to the C++14 standard. Try to document any changes recommended for C++17.

### Notes

- Just because a guideline is listed in this document, does not mean that it's a *good* guideline (or a *bad* one for that matter).
- Some of the guidelines described herein are established primarily by precedence from the history of this code.

### Available Tools

Here are some links to available tools for style conformance. Unfortunately they don't seem perfectly up to the job by themselves so it's still necessary to be aware of the guidelines:

- [`.editorconfig`](../.editorconfig): Configuration file for [EditorConfig](http://editorconfig.org).
- [`.clang-format`](../.clang-format): Configuration file for clang-format.

## Formatting

### General Formatting Guideline For Pre-existing Code
Generally speaking, don't change the formatting of pre-existing code.

### Code Indentation
Code indentation should be done using 4-spaces for every level of code indentation desired. Tabs should not be used for this.

### Class Head Names Begin With Capitalized Letter
Both `class` and `struct` class definition names should begin with a capitalized letter.

## Naming

### Use More Self-Explanatory Names For More Widely Used Definitions
Globally accessible definitions should generally be composed of full words that explain them. Definitions used in smaller contexts can be increasingly tersely named and abbreviated.

### Conjoin Multi Word Names With The First Letter Of New Words Capitalized
Unless the definition is meant to be replaceable with a Standard Library definition, names should be camel-case separated. Definitions meant to be replaceable with Standard Library definitions meanwhile should mimic the naming style of the definitions they're meant to be interchangeable with.

### Start All Include Guard Macros With `PLAYRHO_`
This &mdash; in a sense &mdash; keeps the include guard macros in their own namespace.

### End All Include Guard Macros For `.hpp` Files With `_HPP`

### Capitalize All Letters Of Macro Names
So for example a macro called "one" would be spelled like `ONE`.

### Separate Words In Macro Names With Underscores
Thus a macro called "this is a macro" would appear as: `THIS_IS_A_MACRO`. Or an include guard might be written as: `PLAYRHO_BODY_TYPE_HPP`.

## Functions

### Function Names Usually Begin With Capitalized Letter
The first letter of every function should be capitalized **unless** the function is intended to be a candidate for replacement by a Standard Library function or method.

### Generally Function Names Start With Their Action
The names of all functions &mdash; whether non-static member functions, static member functions, non-member functions, etc. &mdash; should start with the action that they perform. For example: `Get*`, `Set*`, `Find*`, `Transform*`, `Create*`, `Destroy*`, `Insert*`, `Erase*`

### Boolean Constant Member Function Names May Start With The Question They Answer
For example: `IsSleepingAllowed`, `NeedsFiltering`, or `HasValidToi`.

## Variables

### Begin With A Lower-Case Letter

### Begin `class`-class Member Variable Names With `m_`

## Documentation

### Doxygen Style Comment Documentation Is To Be Used

### Every Definition Should Include Documentation Comments
Code missing Doxygen documentation won't pass all of the Pull Request checks and is unlikely to be accepted.

### Every `class`-class Should Document Its Invariants
This is especially the case for any new classes. Lots of existing `class`-class definitions don't document any invariants but they should. Make it a habit to document invariants in a class definition or use the `struct`-class if the class is not supposed to have any invariants.
