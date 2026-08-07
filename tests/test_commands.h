/*
 * test_commands.h — the non-scenario modes of the test executable.
 *
 * Each returns a process exit status. They are separate from the scenario registry because
 * they are tools rather than assertions: they generate artifacts, measure throughput, or
 * exercise the failure-bundle writer on purpose.
 */
#ifndef DRIFTY_TEST_COMMANDS_H
#define DRIFTY_TEST_COMMANDS_H

int test_run_benchmark(int ticks);
int test_verify_failure_bundle(const char *rootDir);
int test_dump_params(const char *path);
int test_generate_corpus(const char *dir);
int test_dump_corpus_index(const char *path);
int test_measure_sweep(const char *key);
int test_dump_corpus_metrics(const char *path);
int test_dump_corpus_cards(const char *dir);
int test_dump_corpus_sheet(const char *dir);
int test_list_cars(void);
int test_generate_roster(const char *dir);

#endif /* DRIFTY_TEST_COMMANDS_H */
