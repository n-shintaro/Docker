#!/bin/sh
cdls ()
{
    \cd "$@" && ls
}
alias cd="cdls"