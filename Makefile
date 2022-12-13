file_finder = @find . -type f $(1) -not \( -path './venv/*' \)
staged_files = git diff --staged --name-only --diff-filter=d HEAD | grep ${1}

CMAKE_FILES = $(call file_finder,\( -name "*\.cmake" -o -name "CMakeLists\.txt" \))
PY_FILES = $(call file_finder,-name "*\.py")
STAGED_PY_FILES = $(call staged_files, -e "\.py")
STAGED_CMAKE_FILES = $(call staged_files, -e "\.cmake" -e "CMakeLists\.txt")

check: check_format lint

format:
	$(PY_FILES) | xargs black
	$(CMAKE_FILES) | xargs cmake-format -i

check_format:
	$(PY_FILES) | xargs black --check
	$(CMAKE_FILES) | xargs cmake-format --check

lint:
	$(PY_FILES) | xargs pylint --rcfile=.pylintrc
	$(PY_FILES) | xargs mypy --strict

lint_staged:
	$(STAGED_PY_FILES) | xargs --no-run-if-empty pylint --rcfile=.pylintrc
	$(STAGED_PY_FILES) | xargs --no-run-if-empty mypy --strict

format_staged:
	$(STAGED_PY_FILES) | xargs --no-run-if-empty black --check
	$(STAGED_CMAKE_FILES) | xargs --no-run-if-empty cmake-format --check