
# README-formatting

This guide summarizes our C/C++ formatting rules and gives examples you can follow while coding. The repo is auto-formatted with **clang-format**.

- **Base**: LLVM  
- **Indentation**: 4 spaces, no tabs  
- **Braces**: **Allman** (opening brace on a new line)  
- **Column limit**: 100  
- **Pointers**: `int* p;` (attach `*` to the type)  
- **Other**: keep empty lines at the start of blocks, no space after C-style cast, don’t align trailing comments

---

## Quick start

1) Put `.clang-format` at the repo root (see the exact file at the end of this README).  
2) Format a file:
```bash
clang-format -i path/to/file.cc
```
3) Format the whole repo (bash):
```bash
find . -regex '.*\.(c|cc|cpp|h|hpp)$' -exec clang-format -i {} +
```

**VS Code**: Enable “Format on Save”; VS Code will pick up `.clang-format`.  
**CLion/Visual Studio/Xcode**: Use “Reformat Code” / “Format Document.”

---

## 1) Braces — Allman style

**Rule:** Open braces on a **new line** for classes, functions, and control statements.

**Good**
```cpp
class Foo
{
public:
    void run();
};

void Foo::run()
{
    if (ready)
    {
        work();
    }
    else
    {
        fallback();
    }
}
```

**Bad**
```cpp
class Foo {
public:
    void run();
};
void Foo::run() {
    if (ready) {
        work();
    } else {
        fallback();
    }
}
```

---

## 2) Indentation & block spacing

**Rule:** 4-space indent. Empty lines at the start of blocks are preserved.

**Good**
```cpp
void process()
{

    // Intentional spacer line at block start.
    do_step();
}
```

**Bad**
```cpp
void process(){
    do_step();
}
```

---

## 3) Column limit & line breaks

**Rule:** Keep lines under ~100 columns. Break calls/declarations cleanly (we avoid bin-packing).

**Good**
```cpp
void configureEngine(
    const std::string& config_path,
    int max_threads,
    bool enable_cache,
    std::unique_ptr<Policy> policy)
{
    // ...
}

auto result = run_task_with_long_name(
    first_arg,
    second_arg,
    42,
    compute_cost());
```

**Bad**
```cpp
void configureEngine(const std::string& config_path, int max_threads, bool enable_cache, std::unique_ptr<Policy> policy) {}

auto result = run_task_with_long_name(first_arg, second_arg, 42, compute_cost());
```

---

## 4) Constructor initializer lists

**Rules:**  
- One-per-line or all-on-one-line (formatter decides).  
- When wrapped, break **before the comma** and indent by 4.

**Good**
```cpp
Widget::Widget()
    : width_(100)
    , height_(200)
    , title_("Main")
{
}
```

**Bad**
```cpp
Widget::Widget() : width_(100), height_(200), title_("Main")
{}
```

---

## 5) Pointers & references

**Rule:** Attach `*` to the **type**, not the variable; same for `&`.

**Good**
```cpp
int* p = nullptr;
const std::string& name = getName();
```

**Bad**
```cpp
int *p = nullptr;
const std::string & name = getName();
```

---

## 6) Spaces before parentheses

**Rule:** Control statements have a space; function calls do **not**.

**Good**
```cpp
if (count > 0)
{
    run(task);
}
```

**Bad**
```cpp
if(count > 0){
    run ( task );
}
```

---

## 7) Switch/case layout

**Rule:** Case labels are indented; braces follow Allman.

**Good**
```cpp
switch (mode)
{
    case Mode::A:
    {
        do_a();
        break;
    }
    case Mode::B:
    {
        do_b();
        break;
    }
    default:
    {
        do_default();
        break;
    }
}
```

**Bad**
```cpp
switch (mode) {
case Mode::A: { do_a(); break; }
default: do_default(); break;
}
```

---

## 8) Includes

**Rule:** clang-format regroups/sorts includes. Prefer:
1. C/C++ standard headers  
2. Third-party headers  
3. Project headers

**Good**
```cpp
#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "project/foo.h"
#include "project/bar.h"
```

**Bad**
```cpp
#include "project/foo.h"
#include <vector>
#include "project/bar.h"
#include <algorithm>
#include <gtest/gtest.h>
```

---

## 9) Comments & casts

**Rules:**  
- Don’t force-align trailing comments.  
- No space after C-style casts.

**Good**
```cpp
int x = (int)compute(); // cast without extra space
int a = 1; // short note
int long_name = 2; // another note
```

**Bad**
```cpp
int x = (int) compute();            // extra space after cast
int a = 1;                          // aligned
int long_name = 2;                  // aligned
```

---

## Practice checklist

- [ ] All braces on **new lines** (Allman).  
- [ ] 4 spaces; no tabs.  
- [ ] Keep lines under ~100 cols; break args/params per line.  
- [ ] Constructor initializers break **before commas** when wrapped.  
- [ ] `int* p`, `const T& r`.  
- [ ] `if (x)` but `func(x)`.  
- [ ] Don’t align trailing comments.  
- [ ] `(int)x`, not `(int) x`.  
- [ ] Includes grouped and sorted.

---

## Reference: `.clang-format`

```yaml
BasedOnStyle: LLVM
UseTab: Never
IndentWidth: 4
ContinuationIndentWidth: 4
ColumnLimit: 100

# Braces: Allman (brace on new line)
BreakBeforeBraces: Allman
BraceWrapping:
  AfterClass: true
  AfterControlStatement: true
  AfterEnum: true
  AfterFunction: true
  AfterNamespace: true
  AfterStruct: true
  AfterUnion: true
  AfterExternBlock: true
  BeforeCatch: true
  BeforeElse: true
  SplitEmptyFunction: true
  SplitEmptyRecord: true
  SplitEmptyNamespace: true
  IndentBraces: false

# Keep bodies multi-line & readable
AllowShortFunctionsOnASingleLine: None
AllowShortIfStatementsOnASingleLine: false
AllowShortLoopsOnASingleLine: false

# Calls/params readability
BinPackArguments: false
BinPackParameters: false
AlignAfterOpenBracket: Align
AllowAllArgumentsOnNextLine: true
AllowAllParametersOfDeclarationOnNextLine: true

# Constructor initializer lists
ConstructorInitializerAllOnOneLineOrOnePerLine: true
AllowAllConstructorInitializersOnNextLine: true
BreakConstructorInitializers: BeforeComma
ConstructorInitializerIndentWidth: 4

# Pointer style
DerivePointerAlignment: false
PointerAlignment: Left

# Includes
SortIncludes: true
IncludeBlocks: Regroup

# Requested options
KeepEmptyLinesAtTheStartOfBlocks: true
SpaceAfterCStyleCast: false
AlignTrailingComments: false

# Misc quality-of-life
ReflowComments: true
FixNamespaceComments: true
SpaceBeforeParens: ControlStatements
IndentCaseLabels: true
```