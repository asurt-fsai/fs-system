#!/usr/bin/env python
"""
This script is used to check if the commit message adheres to the conventional commit rules.
    1. Optional "revert:" tag before commit msg
    2. A type from the config followed by a semi colon and a whitespace
    3. A commit body within a min and max length
Commit-msg.config.json is used to configure the script.
"""

import re
import json
import sys


def checkCommitMsg() -> None:
    """
    Checks the commit message for the conventional commit rules.
    """

    with open(sys.argv[1], encoding="utf-8") as msg:
        commitMsg = msg.read()

    with open("./.hooks/commit-msg.config.json", encoding="utf-8") as configFile:
        config = json.load(configFile)

    regexp = r"^(revert: )?("
    for msgType in config["types"]:
        regexp += f"{msgType}|"
    regexp = regexp[0:-1] + f"): .{{{config['length']['min']},{config['length']['max']}}}$"

    if re.match(regexp, commitMsg) is None:
        print("please enter a commit message in the conventional format.")
        print("commit message must follow (revert: )?<type>: <commit message>")
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    checkCommitMsg()
