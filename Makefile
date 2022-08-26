PACKAGE := $(shell basename $(CURDIR))
export JULIA_PROJECT = $(CURDIR)
export JULIA_DEPOT_PATH = $(realpath $(CURDIR)/jl_depot)

JL := julia

all: README.md

README.md: src/*
	julia --project -e "using $(PACKAGE); $(PACKAGE).readme_docs_generate()"

.PHONY: test
test:
	$(JL) test/runtests.jl

.PHONY: jl
jl:
	$(JL)
