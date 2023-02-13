file_finder = @find . -type f $(1) -not \( -path './venv/*' \)
staged_files = git diff --staged --name-only --diff-filter=d HEAD | grep ${1}
cached_items = @find . -name __pycache__ -or -name "*.pyc" -or -name "*.pyo" -or -name "*_cache"
uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))
file_to_package_finder = $(shell \
	dir=$(dir ${1}); \
	while [ "$$dir" != "." ]; do \
		if [ -f "$$dir/CMakeLists.txt" ]; then \
			echo $$dir; \
			break; \
		fi; \
		dir=$$(dirname $$dir); \
	done; \
)

CMAKE_FILES = $(call file_finder,\( -name "*\.cmake" -o -name "CMakeLists\.txt" \))
PY_FILES = $(call file_finder,-name "*\.py")
CACHED_ITEMS = $(call cached_items)
STAGED_PY_FILES = $(call staged_files, -e "\.py")
STAGED_CMAKE_FILES = $(call staged_files, -e "\.cmake" -e "CMakeLists\.txt")

CURRENT_DIRECTORY := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WORK_SPACE := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../)

DIRS := $(wildcard */)
DIRS += $(shell find $(DIRS) -type d)
PACKAGES_PATH = $(foreach d,$(DIRS),$(if $(wildcard $(d)/CMakeLists\.txt),$(d),))
PACKAGES_NAME = $(foreach pkg,$(PACKAGES_PATH),$(shell basename $(pkg)))

STAGED_FILES = $(shell git diff --staged --name-only --diff-filter=d HEAD)
STAGED_PACKAGES_PATH = $(call uniq, $(foreach file,$(STAGED_FILES),$(call file_to_package_finder, $(file))))
STAGED_PACKAGES_NAME = $(foreach pkg,$(STAGED_PACKAGES_PATH),$(shell basename $(pkg)))

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
	$(STAGED_PY_FILES) | xargs --no-run-if-empty mypy --strict --follow-imports=silent

format_staged:
	$(STAGED_PY_FILES) | xargs --no-run-if-empty black --check
	$(STAGED_CMAKE_FILES) | xargs --no-run-if-empty cmake-format --check

build:
	@cd $(WORK_SPACE); \
	catkin build $(packages);

build_staged: packages=$(STAGED_PACKAGES_NAME)
build_staged: build

rebuild: clean_catkin build

clean_catkin:
	@cd $(WORK_SPACE); \
	rm -r devel build logs;

clean_python:
	$(CACHED_ITEMS) | xargs rm -rf

clean: clean_catkin clean_python
