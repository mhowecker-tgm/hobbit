#!/bin/bash

for i in `rosnode list`; do rosnode kill $i; done
