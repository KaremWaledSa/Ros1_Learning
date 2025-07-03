#!/bin/bash

# Add all changes
git add .

# Commit with a message you provide as argument, or default message
commit_msg=${1:-"Update changes"}
git commit -m "$commit_msg"

# Push to the current branch
git push origin $(git rev-parse --abbrev-ref HEAD)
