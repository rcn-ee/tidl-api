
all: ocl_wrapper.dsp_h

include ../make.inc

CLOCL_FLAGS = -I../ -t


ocl_wrapper.dsp_h: ocl_wrapper.cl
	@echo Generating $@
	@$(CLOCL) $(CLOCL_FLAGS) $^

clean::
	@$(RM) *.obj *.out *.dsp_h
