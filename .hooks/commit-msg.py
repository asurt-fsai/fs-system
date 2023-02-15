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
from typing import Dict, Any


def isValidCommitMsg(commitMsg: str, config: Dict[str, Any]) -> bool:
    """
    Processes the commit message to make sure it adheres to the conventional commit rules.

    parameters
    ----------
    commitMsg: str
        The commit message to be checked
    config: Dict[str, Any]
        The configuration for the script
        in form of a dictionary
        it should contain the following keys:
            types: List[str]
                A list of valid commit types
            length: Dict[str, int]
                A dictionary containing the min and max length of the commit message

    returns
    -------
    bool
        True if the commit message is valid
        False if the commit message is not valid
    """

    minLength: int = config["length"]["min"]
    maxLength: int = config["length"]["max"]

    mergeRegex = r"^Merge branch .*$"
    commitRegex = (
        r"^(?P<revert>revert: )?(?P<type>" + r"|".join(config["types"]) + r"): (?P<message>.*$)"
    )

    if re.match(mergeRegex, commitMsg) is not None:
        return True

    match = re.match(commitRegex, commitMsg)

    if match is None:
        print("Commit doesn't start with revert: or a valid type")
        print("valid types are: " + ", ".join(config["types"]))
        print("commit message must follow (revert: )?<type>: <commit message>")
        print("Make sure to include a space after the colon")
        print('Merge commits must start with "Merge branch"')
        return False

    if len(match.group("message")) < minLength or len(match.group("message")) > maxLength:
        print("Commit message doesn't conform to the length requirements")
        print(f"Message must be between {minLength} and {maxLength}")
        return False

    return True


def checkCommitMsg() -> None:
    """
    Checks the commit message for the conventional commit rules.
    It doesnt return anything, but exits with a 0 if the commit message is valid
    and a 1 if it is not.
    """

    with open(sys.argv[1], encoding="utf-8") as msg:
        commitMsg = msg.read()

    with open("./.hooks/commit-msg.config.json", encoding="utf-8") as configFile:
        config = json.load(configFile)

    if isValidCommitMsg(commitMsg, config):
        sys.exit(0)
    sys.exit(1)


if __name__ == "__main__":
    checkCommitMsg()
