errhead = "\033[91m"
errtail = "\033[0m"

try:
    from mypy import api
except ImportError:
    print(errhead + "Install mypy: py -m pip install mypy" + errtail)
    raise ImportError


def test_mypy():
    result = api.run([".."])  # the path is in FRC-205/src/tests

    if result[0]:
        print("\nType checking report:\n\n" + errhead + result[0] + errtail)
        # print(result[0])  # stdout

    if result[1]:
        print("\nError report:\n")
        print(result[1])  # stderr

    print("\nExit status:", result[2])
    assert result[2] == 0  # 0 on mypy sucess
