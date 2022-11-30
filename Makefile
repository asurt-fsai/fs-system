file_finder = @find . -type f $(1) -not \( -path './venv/*' \)

CMAKE_FILES = $(call file_finder,\( -name "*.cmake" -o -name "CMakeLists.txt" \))
PY_FILES = $(call file_finder,-name "*.py")

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
