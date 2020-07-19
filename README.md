# Diamond Auto: An Autonomous Application Based On Cyber RT

![Action Status](https://github.com/HotzoneAuto/diamond-auto/workflows/Diamond/badge.svg)
## Environment Setup

### Code Clone

Clone master branch:  `git clone git@github.com:HotzoneAuto/diamond-auto.git`

### setup Docker Container

```bash
bash docker/scripts/cyber_start.sh
```

```bash
bash docker/scripts/cyber_into.sh
```

## Build

build  cyber


```bash
bazel build //cyber/...
```

build all modules


```bash
bash apollo.sh build
```


## Delelopment ToolChain

Protobuf file generation :

```bash
python3 scripts/proto_build_generator.py modules/canbus/proto/BUILD

```

### Coding style

**C/C++ coding style**: Apollo adopted the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Make sure your code conforms to this style guide. You can use command `bash apollo.sh lint` to check if your code has any style problem. Or
C++ Code format:

```bash
bash scripts/clang-format.sh
```

**Python coding style**:  Apollo adopted the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html). You can use the  [yapf](https://github.com/google/yapf) command `yapf -i --style='{based_on_style: google}' foo.py` to format a file foo.py.
