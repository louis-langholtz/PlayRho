# PlayRho Library Style Guide

This is the style guide for the PlayRho library. This guide is meant for contributors who want to contribute pull requests for code changes.

For general information on contributing, see the [Contributing](../CONTRIBUTING.md) document.

## General Guidelines

### Follow The C++ Core Guidelines For Anything Not Described Herein
For anything not described herein, please follow the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md).

### C++17 Is The Applicable Standard
Write code to the C++17 standard. Try to document any changes recommended for C++20.

### Notes

- Just because a guideline is listed in this document, does not mean that it's a *good* guideline (or a *bad* one for that matter).
- Some of the guidelines described herein are established primarily by precedence from the history of this code.

### Use of `auto`
Prefer to use `auto` **unless** it makes it harder for you to do what you're trying to do. Don't remove the use of `auto` from code unless it's code you've contributed.

## Formatting

### General Formatting Guideline For Pre-existing Code
Generally speaking, don't change the formatting of pre-existing code.

### Indentation
Indent code using 4-spaces for every level of code indentation desired. Tabs may not be used for this.

#### Namespaces
Don't add any levels of indentation within namespaces. For example, use the following style for namespaces and any declarations within them:
```
namespace playrho {

struct MyData;

namespace d2 {

int MyNewFunctionForD2(const MyData& arg);

} // namespace d2
} // namespace playrho
```

#### Class Definitions
Don't indent class access specifiers (`public:`, `protected:`, and `private:`).
I.e., use the following formatting style:
```
class MyNewClass {
public:
    MyNewClass() = default;
    // ... whatever
private:
    int m_data;
};
```

### Line Wrapping
This speaks to when to wrap source code to the next line.
Use a column width of 100.

For namespaces, if-statements, switch-statements, case-labels, and class definitions,
wrap to the next line **after** the `{` character.
For function definitions, prefer to wrap to the next line **before** the `{` character.

Here's what this looks like:
```
namespace Test { // ClangFormat: "AfterNamespace: false"

class MyClass { // ClangFormat: "AfterClass: true"
public:
    MyClass(int foo): m_data{foo}
    {
        // Nothing done here in function block.
    }
private:
    int m_data;
};

struct MyStruct { // ClangFormat: "AfterStruct: false"
  int wow;
};

void DoSomething(bool someBool, int val) // ClangFormat: "AfterFunction: true"
{
    if (someBool) {
        myFunction();
    } // ClangFormat: "BeforeElse: true"
    else {
        someOtherFunction();
    }
    switch (val) {
    case 0: { // ClangFormat: "AfterCaseLabel: false"
        int foo = 2;
        doIt(foo);
        break;
    }
    case 1:
        break;
    default:
        break;
    }
}

} // namespace Test
```

### Spaces

#### Operator Overloads
Make operator overload definitions look like function definitions.
Don't put any spaces between the specification of the operator and its parameters:
```
constexpr Acceleration operator-(const Acceleration& lhs, const Acceleration& rhs)
{
    return Acceleration{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
}

constexpr Acceleration Subtract(const Acceleration& lhs, const Acceleration& rhs)
{
    return Acceleration{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
}
```

### Available Tools

Here are some links to available tools to help with formatting. Unfortunately they don't seem perfectly up to the job by themselves so it's still necessary to be aware of these guidelines:

- [`.editorconfig`](../.editorconfig): Configuration file for [EditorConfig](http://editorconfig.org).
- [`.clang-format`](../.clang-format): Configuration file for [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html). From a POSIX shell, ClangFormat can be run as: `clang-format -i PlayRho/**/*.cpp PlayRho/**/*.hpp`.

## Naming

### Use More Self-Explanatory Names For More Widely Used Definitions
Compose globally accessible definitions with full words that explain them. Definitions used in smaller contexts can be increasingly tersely named and abbreviated.

### Class Names Begin With Capitalized Letter
Both `class` and `struct` class definition names are to begin with a capitalized letter.

### Conjoin Multi Word Names With The First Letter Of New Words Capitalized
Unless the definition is meant to be replaceable with a Standard Library definition, names are to be camel-case separated. Definitions meant to be replaceable with Standard Library definitions meanwhile are to mimic the naming style of the definitions they're meant to be interchangeable with.

### Start All Include Guard Macros With `PLAYRHO_`
This &mdash; in a sense &mdash; keeps the include guard macros in their own namespace.

### End All Include Guard Macros For `.hpp` Files With `_HPP`

### Capitalize All Letters Of Macro Names
So for example a macro called "one" would be spelled like `ONE`.

### Separate Words In Macro Names With Underscores
Thus a macro called "this is a macro" would appear as: `THIS_IS_A_MACRO`. Or an include guard might be written as: `PLAYRHO_BODY_TYPE_HPP`.

## Functions

### Function Names Usually Begin With Capitalized Letter
Capitalize the first letter of every function **unless** the function is intended to be a candidate for replacement by a Standard Library function.

### Generally Function Names Start With Their Action
Start the names of all functions &mdash; whether non-static member functions, static member functions, non-member functions, etc. &mdash; with the action that they perform. For example: `Get*`, `Set*`, `Find*`, `Transform*`, `Create*`, `Destroy*`, `Insert*`, `Erase*`

### Boolean Constant Member Function Names May Start With The Question They Answer
For example: `IsSleepingAllowed`, `NeedsFiltering`, or `HasValidToi`.

## Variables

### Begin With A Lower-Case Letter

### Begin `class`-class Member Variable Names With `m_`

## Documentation

### Doxygen Style Comment Documentation Is To Be Used

### Include Documentation Comments With Every Declaration
Code missing Doxygen documentation won't pass all of the Pull Request checks and is unlikely to be accepted.

### Document the Invariants For Every `class`-class
This is especially the case for new classes. Lots of existing `class`-class definitions don't document any invariants but I'd like them to. Make it a habit to document invariants in a class definition or use the `struct`-class if the class is not supposed to have any invariants.

## Classes

### Use `struct` For C++ Classes That Have No Invariants
Declare classes that have no invariants using the `struct` *class-key*.

### Use `class` For C++ Classes That Have One Or More Invariants
Classes that have one or more invariants are to be declared using the `class` *class-key*.

### Document Invariants Using `@invariant` Doxygen Command
PlayRho library classes that are declared using the `class` *class-key* should have one or more `@invariant` Doxygen commands in their class comments.
