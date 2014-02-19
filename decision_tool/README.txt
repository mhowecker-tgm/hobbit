If you encounter linking errors related to libdecisiontool.so when compiling the decision_tool_tester package:
1. Compile this package to produce the libdecisiontool.so file
2. Copy (and replace) libdecisiontool.so from devel/lib folder to src/decision_tool_tester/src folder
3. decision_tool_tester should compile and link successfully