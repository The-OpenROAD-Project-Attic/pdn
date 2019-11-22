TESTS=\
    gcd

.PHONY: $(TESTS)

all: $(TESTS)

clean.% : 
	-@$(MAKE) -C $* clean 

check.% : 
	@$(MAKE) -C $* check 

test.% : 
	@$(MAKE) -C $* test 

approve.% :
	@$(MAKE) -C $* approve 

clean: $(foreach block,$(TESTS),clean.$(block))
	@echo "Clean complete"

check: $(foreach block,$(TESTS),check.$(block))
	@echo "Checks complete"
        
$(TESTS) :
	$(MAKE) -C $@ 
