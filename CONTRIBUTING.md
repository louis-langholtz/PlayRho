# Contributing to PlayRho

:+1::tada: First off, thank you for considering contributing to the
PlayRho project! :tada::+1:

There are just some guidelines and rules that you need to follow. These are
explained in this document.

## How Can I Contribute?

You can contribute by using the Issues interface and by possibly additionally
using the Pull requests interface...

### Reporting Bugs or Suggesting Enhancement Via the Issues Interface

The [Issues interface](https://github.com/louis-langholtz/PlayRho/issues) is a place
to begin with contributing. Use it for reporting bugs or suggesting enhancements
via the following steps:

1. Check [the existing issues](https://github.com/louis-langholtz/PlayRho/issues?q=is%3Aissue)
  to see if an issue has already been filed for what you want.
2. If there's an existing issue that's been filed for what you want, consider
  adding your reaction to it and/or adding a comment to it especially if you have something
  helpful you can add to it.
3. If there's not already an existing issue for what you want, consider opening
  a new issue.

### Submitting File Changes Using the Pull Requests Interface

The [Pull requests interface](https://github.com/louis-langholtz/PlayRho/pulls)
is the interface to use *after* you've started a contribution using the Issues interface,
and you want to additionally contribute by offering file changes.

While this may be the most helpful way to contribute, it's also
more cumbersome on prospective contributors as it has more formal requirements.

For starters...

- Pull requests will not be accepted for files that are licensed
  under any other license than the [zlib License](https://opensource.org/licenses/Zlib).
  This project is licensed under the zlib License and at least for simplicity sake,
  I'd like to keep it all that way.

- Pull requests will not be accepted from users who have not first agreed to the
  PlayRho Project's [Contributor License Agreement](#contributor-license-agreement).

Steps to follow for making pull requests:

1. Make sure that there's an open issue describing the bug or
   feature you're intending to fix - creating a new issue for this if needed.
   Even if you think the change is relatively minor,
   it's helpful to know what people are working on.  Mention in the initial
   issue that you are planning to work on that bug or feature so that it can
   be assigned to you.

1. Follow the normal process of [forking][] the project, and setup a new
   branch to work in.  It's important that each group of changes be done in
   separate branches in order to ensure that a pull request only includes the
   commits related to that bug or feature.

1. When changing files, as a general rule of thumb, continue using the
   same style and format that the file is already using. For new C++ files,
   use `clang-format` and/or the project's `.clang-format` file.
   For more specific guidelines, see the [Style Guides](#style-guides) section
   of this document.

1. Do your best to have [well-formed commit messages][] for each change.
   This provides consistency throughout the project, and ensures that commit
   messages are able to be formatted properly by various git tools.

1. Finally, push the commits to your fork and submit a [pull request][].

[forking]: https://help.github.com/articles/fork-a-repo
[well-formed commit messages]: http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html
[pull request]: https://help.github.com/articles/creating-a-pull-request

## Contributor License Agreement ##

[![CLA assistant](https://cla-assistant.io/readme/badge/louis-langholtz/PlayRho)](https://cla-assistant.io/louis-langholtz/PlayRho)

Contributions must be accompanied by a Contributor
License Agreement.  This is not a copyright **assignment**, it simply gives
permission to use and redistribute your contributions as part of the
project. Whether you are an individual writing original source code that you're
sure you own the intellectual property, or you work for a company that wants to
allow you to contribute your work, you'll need to sign the [CLA][].

You generally only need to agree to the CLA once (unless it changes), so if
you've already submitted one (even if it was for a different project), you
probably don't need to do it again.

[CLA]: https://cla-assistant.io/louis-langholtz/PlayRho

## Style Guides

### Git Commit Messages

* Use the present tense ("Add feature" not "Added feature").
* Prefer the imperative mood ("Move cursor to..." not "Moves cursor to...").
* Limit the first line to 72 characters or less.
* Reference issues and pull requests liberally after the first line.
* When only changing documentation, include `[ci skip]` in the commit description.
* Consider starting the commit message with an applicable emoji:
    * :art: `:art:` when improving the format/structure of the code.
    * :racehorse: `:racehorse:` when improving performance.
    * :non-potable_water: `:non-potable_water:` when plugging memory leaks.
    * :memo: `:memo:` when writing docs.
    * :penguin: `:penguin:` when fixing something on Linux.
    * :apple: `:apple:` when fixing something on macOS.
    * :checkered_flag: `:checkered_flag:` when fixing something on Windows.
    * :bug: `:bug:` when fixing a bug.
    * :fire: `:fire:` when removing code or files.
    * :green_heart: `:green_heart:` when fixing the CI build.
    * :white_check_mark: `:white_check_mark:` when adding tests.
    * :lock: `:lock:` when dealing with security.
    * :arrow_up: `:arrow_up:` when upgrading dependencies.
    * :arrow_down: `:arrow_down:` when downgrading dependencies.
    * :shirt: `:shirt:` when removing linter warnings.

### C++ Style Guide

Generally speaking, follow the style of the existing code.
