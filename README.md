# Diamond Auto: An Autonomous Application Based On Cyber RT

![Action Status](https://github.com/HotzoneAuto/diamond-auto/workflows/Diamond/badge.svg)

### On site commissioning
![20201117175018](https://user-images.githubusercontent.com/45028297/99381225-e811fb80-2905-11eb-9c0c-180431737ad0.jpg)
<!--![20201117175036](https://user-images.githubusercontent.com/45028297/99381315-04159d00-2906-11eb-98b2-9d95c7cc4ab6.jpg)-->
![20201117175058](https://user-images.githubusercontent.com/45028297/99381331-07a92400-2906-11eb-99d5-4226330cc1d9.png)

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
bash apollo.sh build_gpu
```


## Development ToolChain

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
