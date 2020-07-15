# Diamond Auto: An Autonomous Application Based On Cyber RT

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


## ToolChain 

C++ Code format:

```bash
bash scripts/clang-format.sh
```

Protobuf file generation :

```bash
python3 scripts/proto_build_generator.py modules/canbus/proto/BUILD

```
