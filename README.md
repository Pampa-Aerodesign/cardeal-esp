<img src="img/logo.png?raw=true" width="200" />

# Cardeal ESP

This is the repository for Pampa Aerodesign's first continuously developed embedded system, cardeal-esp.

We're currently using esp-idf version v4.2.

## Cloning the repo

This repo contains some git submodules, so you need extra steps when cloning the repo to local:

1. git clone https://github.com/Pampa-Aerodesign/cardeal-esp.git
2. git submodule init
3. git submodule update

Git submodules are basically repositories embedded inside another repository. In our case, some ESP components from the GitHub community are referenced this way.

## Code formatting

The cardeal-esp project is configured with a code formatting tool called clang-format. It is avaliable standalone or with the [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) in Visual Studio Code.

This repo is configured with a GitHub action that automatically formats code in the master branch when a commit or a pull request is made to such branch **(currently disabled)**. It ignores files in the `components` folder to avoid changing those source files.

The `.clang-format` file in the root directory contains the configuration for the code style used.

To prevent a section of code from being formatted, one can use `// clang-format off` and `// clang-format on` around it.
