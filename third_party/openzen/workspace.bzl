"""Loads the openzen library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # openzen
    native.new_local_repository(
        name = "openzen",
        build_file = clean_dep("//third_party/openzen:openzen.BUILD"),
        path = "/opt/apollo/pkgs/OpenZenRelease/include",
    )
