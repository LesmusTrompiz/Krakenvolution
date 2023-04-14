#!/bin/bash

filename="$1"
min_delay="$2"
max_delay="$3"

if [[ -z "$filename" || -z "$min_delay" || -z "$max_delay" ]]; then
  echo "Usage: $0 <filename> <min_delay in seconds> <max_delay in seconds>"
  exit 1
fi

if [[ ! -f "$filename" ]]; then
  echo "Error: File '$filename' does not exist."
  exit 1
fi

if (( $(echo "$min_delay > $max_delay" | bc -l) )); then
  echo "Error: Min delay should not be greater than max delay."
  exit 1
fi

while read -r line; do
  echo "$line"
  random_decimal=$(echo "scale=4; $min_delay + ($max_delay - $min_delay) * $RANDOM / 32767" | bc -l)
  sleep "$random_decimal"
done < "$filename"
