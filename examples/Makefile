.SILENT:

# All other examples
MFS  = $(wildcard */Makefile)
DIRS = $(patsubst %/Makefile,%,$(MFS))

define make_in_dirs
	for dir in $(1); do \
	  echo "=============== " $$dir " =================" ; \
	  $(MAKE) --no-print-directory $(MAKEFILE) -C $$dir $(2); \
	done
endef


all:
	$(call make_in_dirs, $(DIRS), )

.PHONY: test
test:
	$(call make_in_dirs, $(DIRS), test)

.PHONY: cross
cross: all

.PHONY: clean
clean:
	$(call make_in_dirs, $(DIRS), clean)

