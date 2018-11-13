#!/usr/bin/env python
function=None
def test():
	global function
	def another_function():
		print("ok")
	function=another_function
	function()
def main():
	test()
	function()

main()