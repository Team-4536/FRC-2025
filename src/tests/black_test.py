import subprocess

errhead = "\033[91m"
errtail = "\033[0m"

try:
    import black
except ImportError:
    print(errhead + "Install black: py -m pip install black" + errtail)
    raise ImportError


def test_black():
    # black doesn't have an api for some reason so this will do
    ret = subprocess.call(
        "py -m black --check ../"
    )  # doesnt reformat, just checks if files are formatted
    if ret == 1:  # 0 on success
        print(errhead + "Your code is not formatted with balck." + errtail)
    assert ret == 0
