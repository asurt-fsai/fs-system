SHELL := /bin/bash

file_finder = @find . -type f $(1) -not \( -path './venv/*' \)
staged_files = git diff --staged --name-only --diff-filter=d HEAD | grep ${1}
cached_items = @find . -name __pycache__ -or -name "*.pyc" -or -name "*.pyo" -or -name "*_cache"
run_unittest = python -m unittest discover -s ${1} -p "*_test.py";
run_rostest = @rostest ${1} ${2}

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
package_to_test_finder = $(shell find . -type d -path "*/$(strip ${1})/tests")
if_not_exists = if [ -z "$(strip ${1})" ]; then \
				${2} \
			else \
				${3} \
			fi


CMAKE_FILES = $(call file_finder,\( -name "*\.cmake" -o -name "CMakeLists\.txt" \))
PY_FILES = $(call file_finder,-name "*\.py")
CACHED_ITEMS = $(call cached_items)
STAGED_PY_FILES = $(call staged_files, -e "\.py\>")
STAGED_CMAKE_FILES = $(call staged_files, -e "\.cmake" -e "CMakeLists\.txt")

CURRENT_DIRECTORY := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WORK_SPACE := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../)

TEST_DIRECTORIES = $(foreach d,$(DIRS),$(if $(wildcard $(d)/tests),$(CURRENT_DIRECTORY)/$(d)/tests,))
ROS_TESTS_DIRECTORIES = $(foreach d,$(DIRS),$(if $(wildcard $(d)/launch),$(d)/launch,))

DIRS := $(wildcard */)
DIRS += $(shell find $(DIRS) -type d)
PACKAGES_PATH = $(foreach d,$(DIRS),$(if $(wildcard $(d)/CMakeLists\.txt),$(d),))
PACKAGES_NAME = $(foreach pkg,$(PACKAGES_PATH),$(shell basename $(pkg)))

STAGED_FILES = $(shell git diff --staged --name-only --diff-filter=d HEAD)
STAGED_PACKAGES_PATH = $(call uniq, $(foreach file,$(STAGED_FILES),$(call file_to_package_finder, $(file))))
STAGED_PACKAGES_NAME = $(foreach pkg,$(STAGED_PACKAGES_PATH),$(shell basename $(pkg)))
STAGED_TESTS_DIRECTORIES = $(foreach pkg,$(STAGED_PACKAGES), $(if $(wildcard $(pkg)/tests),$(pkg)/tests,))
STAGED_ROS_TESTS_DIRECTORIES = $(foreach pkg,$(STAGED_PACKAGES), $(if $(wildcard $(pkg)/launch),$(pkg)/launch,))

default: help

##@ Help
.PHONY: help
help: ## Display information about the available commands.
	@awk 'BEGIN {FS = ":.*##|##"; printf "\nUsage:\n  make \033[34m<target> \033[36marg1=\"\" \033[36marg2=\"\"\033[0m\n"} /^[a-zA-Z_0-9-]+:.*?##/ { printf "  \033[33m%-15s\033[0m %s\n", $$1, $$2 } /^[[:space:]]*## .*/ { printf "  %-15s\033[33m\033[0m %s\n", $$100, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)


##@ Format and lint

check: check_format lint ## Runs formating and linting checks without modifying any files.

format: ## Runs formating on python and cmake files.
	$(PY_FILES) | xargs --no-run-if-empty black
	$(CMAKE_FILES) | xargs --no-run-if-empty cmake-format -i

check_format: ## Checks if python and cmake files are formated.
	$(PY_FILES) | xargs --no-run-if-empty black --check
	$(CMAKE_FILES) | xargs --no-run-if-empty cmake-format --check

lint: ## Checks if python files are linted using pylint and mypy.
	$(PY_FILES) | xargs --no-run-if-empty pylint --rcfile=$(CURRENT_DIRECTORY)/.pylintrc
	$(PY_FILES) | xargs --no-run-if-empty mypy --strict

lint_staged: ## Checks if staged python files are linted using pylint and mypy.
	$(STAGED_PY_FILES) | xargs --no-run-if-empty pylint --rcfile=$(CURRENT_DIRECTORY)/.pylintrc
	$(STAGED_PY_FILES) | xargs --no-run-if-empty mypy --strict --follow-imports=silent

format_staged: ## Runs formating on staged python and cmake files.
	$(STAGED_PY_FILES) | xargs --no-run-if-empty black
	$(STAGED_CMAKE_FILES) | xargs --no-run-if-empty cmake-format

##@ Build

build: 	## Runs catkin build on all packages.
		## Accepts packages as argument.
		## e.g. make build packages="package1 package2 package3"
	@cd $(WORK_SPACE); \
	catkin build $(packages);

build_staged: packages=$(STAGED_PACKAGES_NAME) ## Runs catkin build on staged packages.
build_staged: build

rebuild: clean_catkin build ## Runs catkin clean and build on all packages.
							## Accepts packages as argument.
							## e.g. make rebuild packages="package1 package2 package3"

##@ Test

test: build ## Builds packages and run its tests
			## Accepts packages as argument.
			## e.g. make test packages="package1 package2 package3"
	source $(WORK_SPACE)/devel/setup.bash; \
	set -e ; \
	$(call if_not_exists, $(packages),\
		$(foreach pkg,$(TEST_DIRECTORIES),  echo Running tests of $(pkg);$(call run_unittest, $(pkg))), \
		echo;\
		$(foreach pkg,$(packages), echo Running tests of $(pkg);\
		$(call run_unittest, $(call package_to_test_finder, $(pkg))))\
	)

test_staged: packages=$(STAGED_PACKAGES_NAME) ## Builds staged packages and run its tests.
test_staged: test

##@ Clean

clean_catkin: ## Cleans catkin workspace. Removes devel, build and logs directories.
	catkin clean

clean_python: ## Removes all python cache files.
	$(CACHED_ITEMS) | xargs rm -rf

clean: clean_catkin clean_python ## Runs catkin build, devel, logs. and removes all python cache files.
