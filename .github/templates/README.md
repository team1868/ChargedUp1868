## Label template directory 
This directory contains templates used in the `add-template-to-issue.yml` workflow

Each template must have a filename of `label-<name of label>.md` to automatically be applied to a label of that name

For example, if you would like to add a template to an issue when the `badbug` label is applied, add a file in this directory called

`label-badbug.md`

The template will be added as a new comment as to avoid muddling existing comments
