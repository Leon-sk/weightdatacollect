#! /bin/sh

supervisorctl stop weight
/opt/weight_test/build/weight camParam
supervisorctl start weight

