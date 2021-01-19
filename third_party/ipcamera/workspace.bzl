"""Loads the ipcamera library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "ipcamera",
        build_file = clean_dep("//third_party/ipcamera:ipcamera.BUILD"),
        path = "/opt/apollo/pkgs/ipcamera/include",
    )
