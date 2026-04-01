# Spatz replication – build & test automation

PYTHON  ?= python3
PYTEST  ?= $(PYTHON) -m pytest
PYLINT  ?= $(PYTHON) -m pylint
SRCDIR  := src
TESTDIR := tests

.PHONY: all test lint clean help

all: test

## Run the full test suite
test:
	$(PYTEST) $(TESTDIR) -v

## Run tests with coverage report
coverage:
	$(PYTEST) $(TESTDIR) -v --cov=$(SRCDIR) --cov-report=term-missing

## Lint the source code
lint:
	$(PYLINT) $(SRCDIR)/riscv --errors-only

## Remove Python caches
clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true
	rm -rf .pytest_cache .coverage htmlcov

## Show this help
help:
	@grep -E '^## ' Makefile | sed 's/## /  /'
