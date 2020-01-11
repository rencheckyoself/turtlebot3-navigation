# Compile the program
g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp

# Send the output to the answer file
# ./rigid2d_test < test1_input.txt > test1_answers.txt

# Test the program with the known inputs to see if the output matches the known output
./rigid2d_test < test1_input.txt | cmp - test1_answers.txt && echo "Success!" || echo "Failed!"
