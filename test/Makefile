TESTS=\
    gcd

.PHONY: $(TESTS)

all: $(TESTS)

clean.% : 
	-@$(MAKE) -C $* clean BLOCK=$*

check.% : 
	@$(MAKE) -C $* check BLOCK=$*

test.% : 
	@$(MAKE) -C $* test BLOCK=$*

approve.% :
	@$(MAKE) -C $* approve BLOCK=$*

clean: $(foreach block,$(TESTS),clean.$(block))
	echo "Clean complete"

check: $(foreach block,$(TESTS),check.$(block))
	echo "Checks complete"
        
$(TESTS) :
	$(MAKE) -C $@ BLOCK=$@
