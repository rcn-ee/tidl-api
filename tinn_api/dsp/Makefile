
all: ocl_wrapper.out

include ../make.inc

CLOCL_FLAGS = -I../


ocl_wrapper.out: ocl_wrapper.cl
	@echo Generating $@
	@$(CLOCL) $(CLOCL_FLAGS) $^

clean::
	@$(RM) *.obj *.out
