
default: testSocket testFile testComparator
run: runSocket runFile runComparator


testSocket: 
	cd test_socket; ${MAKE}
runSocket:
	cd test_socket; ${MAKE} run

testFile:
	cd test_file; ${MAKE}
runFile:
	cd test_file; ${MAKE} run

testComparator:
	cd test_comparator; ${MAKE}
runComparator:
	cd test_comparator; ${MAKE} run


clean:
	cd test_socket; ${MAKE} clean
	cd test_file; ${MAKE} clean
	cd test_comparator; ${MAKE} clean

