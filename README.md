# ow
### The VHDL One Wire master & device wrapper that loves you

Are you working with VHDL?  Do you need to implement a One Wire/1-wire master in hardware?  If you answered yes to either, you almost certainly need a hug.  But if you answered yes to both, you *definitely* need a hug.  And this code.  


### So what is ow?

What ow is depends on how you use it.  If you aren't so much interested in dealing with one wire directly and just want to use a one wire device as if it were any other hardware peripheral, then ow can do that - at least for the `DS2431` and `DS2433`.  

But there are many different ways one might create a wrapper even for a humble 1kb EEPROM, and you might not like the way we've done it.  Or maybe you have something more exciting planned and want to create your own wrapper or interface for one of the many other one wire devices available.  In that case, ow can help you too.  

ow makes writing the sequential series of one wire bus transactions needed to do anything with any one wire device as painless and easy to understand as possible, and you can drop in a one wire communication sequence directly into a single state of a state macine.  This is what reading from a `DS2431` looks like using ow:


```vhdl
when read_from_eeprom => -- state in a flat FSM
  ow(bus_reset); -- Reset bus and perform presence detect
  ow(tx => SKIP_ROM); -- 0xCC
  ow(tx => READ_MEMORY); -- 0xFO
  ow(tx => x"00"); -- Target Address 1 (TA1)
  ow(tx => x"00"); -- Target Address 2 (TA2)
  ow(rx => eeprom_buffer); -- std_logic_vector or array of desired read length
  ow(proceed_to => load_contents); -- the state to put the FSM into when done
```


This snippet might be setting off some alarm bells - all of those calls to the ow procedure are going to be executed all at once, during just one clock cycle.  

And this is what makes ow so pleasant to use.  There is no getting around it, using parallel hardware descriptions to describe sequential behavior just sucks.  So ow's solution is: don't.   

ow lets you make parallel calls to it, as many as you wish in a single clock, **but will execute them sequentially, in the order you made the calls in your code anyway.**  Not only will the instructs be performed in order, they will occur only after the previous one has finished.  The only requirement is that you use the `proceed_to` instruction to change state from a state containing calls to ow.  No other special rules need be observed, and there are no surprises.  

The sequential nature of ow is not fragile and handled at time of synthesis, not in hardware.  This ensures that it is robust, well-behaved, and of course, synthesizeable.  You may use it within control structures like if statements and loops.

Let's checkout another example, one where we write to the EEPROM scratch pad then copy the contents to memory:

```vhdl
when write_scratchpad_and_copy_to_eeprom =>
  ow(bus_reset);
  ow(tx => SKIP_ROM); -- 0xCC
  ow(tx => WRITE_SCRATCHPAD); -- 0x0F
  ow(tx => ta1); -- Target Address 1 (TA1)
  ow(tx => ta2); -- Target Address 2 (TA2)
  ow(tx => scratchpad_contents); -- Array of std_logic_vectors
  ow(rx => scratchpad_crc); -- optional
  ow(bus_reset);
  ow(tx => SKIP_ROM); -- 0xCC
  ow(tx => COPY_SCRATCHPAD); -- 0x55
  ow(tx => ta1); -- Target Address 1 (TA1) ──────────────────> ┐
  ow(tx => ta2); -- Target Address 2 (TA2) ──────────────────> ┼──> Aurthorization code
  ow(tx => AUTH_CODE); -- 0x07 byte count of scratchpad ──────> ┘
  ow(wait_for_program); -- Wait 12.5ms, t_prog_max
  ow(proceed_to => idle);
```


Based on Maxim's `ds1wm` [one wire master [PDF]](https://pdfserv.maximintegrated.com/en/ds/DS1WM.pdf).

**NOTE: This is currently under active development.  Neither the documentation or the code itself is currently feature complete.**
