# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# Prepare
apt-get install libbluetooth-dev
git clone https://bitbucket.org/lpresearch/openzen.git
pushd openzen
    git submodule update --init
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DZEN_STATIC_LINK_LIBCXX=On -DZEN_BLUETOOTH=OFF -DCMAKE_INSTALL_PREFIX=/opt/apollo/pkgs/OpenZenRelease ..
    make -j4 install 
popd
