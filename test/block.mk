$(BLOCK)_pdn.def : PDN.cfg
	apply_pdn PDN.cfg

check: $(BLOCK)_pdn.def
	sed -e '1,4d' $< | diff -q - $(BLOCK)_pdn.check

approve: $(BLOCK)_pdn.check

$(BLOCK)_pdn.check: $(BLOCK)_pdn.def
	sed -e '1,4d' $< > $@

clean:
	-@rm dummy.guide
	-@rm macrocell.list
	-@rm run.param
	-@rm pin_dumper.log
	-@rm $(BLOCK)_pdn.def
	-@rm $(BLOCK)_post_T8.def
	-@rm $(BLOCK).geom.rpt
	-@rm floorplan.def.v

