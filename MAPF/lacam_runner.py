from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Protocol

from lacam_adapter import LaCAMProblem, LaCAMSolution, make_failed_solution


class LaCAMBackend(Protocol):
    """LaCAM backend の protocol。

    実際の solver 接続方法は未確定でも、
    `solve(problem, config) -> LaCAMSolution` を満たせば runner から使える。
    """

    def solve(
        self,
        problem: LaCAMProblem,
        config: "LaCAMRunnerConfig",
    ) -> LaCAMSolution:
        """LaCAM 問題を解く。

        Args:
            problem: LaCAM 問題。
            config: 実行設定。

        Returns:
            LaCAM 解。
        """
        ...


@dataclass(frozen=True)
class LaCAMRunnerConfig:
    """LaCAM runner 設定。

    Attributes:
        time_limit_sec: 計算時間制限 [sec]。
        debug: デバッグ出力有効化フラグ。
        seed: 乱数シード。
    """

    time_limit_sec: float | None = None
    debug: bool = False
    seed: int = 42


class ReplayBackend:
    """テスト用 backend。

    あらかじめ用意した `LaCAMSolution` をそのまま返す。
    adapter / runner の配線確認に使う。
    """

    def __init__(
        self,
        solution: LaCAMSolution,
    ) -> None:
        """初期化する。

        Args:
            solution: 固定で返す解。
        """
        self._solution = solution

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """固定解を返す。

        Args:
            problem: LaCAM 問題。
            config: 実行設定。

        Returns:
            固定の LaCAM 解。
        """
        return self._solution


class NotImplementedBackend:
    """未実装 backend。

    実 solver 接続前の明示的な placeholder。
    """

    def solve(
        self,
        problem: LaCAMProblem,
        config: LaCAMRunnerConfig,
    ) -> LaCAMSolution:
        """未実装例外を投げる。

        Args:
            problem: LaCAM 問題。
            config: 実行設定。

        Raises:
            NotImplementedError: 常に送出する。
        """
        raise NotImplementedError(
            "LaCAM backend is not implemented yet. "
            "Inject a concrete backend into LaCAMRunner."
        )


class LaCAMRunner:
    """LaCAM runner 最小実装。

    backend に solver 実行を委譲し、
    runner 自体は timing / exception handling / debug logging を担当する。
    """

    def __init__(
        self,
        backend: LaCAMBackend | None = None,
        config: LaCAMRunnerConfig | None = None,
    ) -> None:
        """初期化する。

        Args:
            backend: 実 solver backend。未指定時は placeholder を使う。
            config: 実行設定。未指定時は default を使う。
        """
        self.backend = backend if backend is not None else NotImplementedBackend()
        self.config = config if config is not None else LaCAMRunnerConfig()

    def solve(
        self,
        problem: LaCAMProblem,
    ) -> LaCAMSolution:
        """LaCAM 問題を解く。

        Args:
            problem: LaCAM 問題。

        Returns:
            LaCAM 解。
        """
        if self.config.debug:
            print(
                "[LaCAMRunner] Start solve"
                f" | agents={problem.num_agents}"
                f" | time_limit_sec={self.config.time_limit_sec}"
                f" | seed={self.config.seed}"
            )

        start_time = time.perf_counter()

        try:
            raw_solution = self.backend.solve(problem=problem, config=self.config)
            elapsed = time.perf_counter() - start_time

            solution = LaCAMSolution(
                solved=raw_solution.solved,
                comp_time_sec=elapsed,
                paths_by_agent=raw_solution.paths_by_agent,
                raw_status=raw_solution.raw_status,
                notes=raw_solution.notes,
            )

            if self.config.debug:
                print(
                    "[LaCAMRunner] Finished"
                    f" | solved={solution.solved}"
                    f" | comp_time_sec={solution.comp_time_sec:.6f}"
                    f" | status={solution.raw_status}"
                )

            return solution

        except TimeoutError:
            elapsed = time.perf_counter() - start_time
            if self.config.debug:
                print(
                    "[LaCAMRunner] Timeout"
                    f" | comp_time_sec={elapsed:.6f}"
                )
            return make_failed_solution(
                comp_time_sec=elapsed,
                raw_status="timeout",
                notes="Solver timeout",
            )

        except NotImplementedError as exc:
            elapsed = time.perf_counter() - start_time
            if self.config.debug:
                print(
                    "[LaCAMRunner] Not implemented"
                    f" | comp_time_sec={elapsed:.6f}"
                    f" | error={exc}"
                )
            return make_failed_solution(
                comp_time_sec=elapsed,
                raw_status="not_implemented",
                notes=str(exc),
            )

        except Exception as exc:  # noqa: BLE001
            elapsed = time.perf_counter() - start_time
            if self.config.debug:
                print(
                    "[LaCAMRunner] Exception"
                    f" | comp_time_sec={elapsed:.6f}"
                    f" | error={exc}"
                )
            return make_failed_solution(
                comp_time_sec=elapsed,
                raw_status="exception",
                notes=str(exc),
            )


def run_lacam(
    problem: LaCAMProblem,
    backend: LaCAMBackend | None = None,
    time_limit_sec: float | None = None,
    debug: bool = False,
    seed: int = 42,
) -> LaCAMSolution:
    """LaCAM を単発実行する convenience 関数。

    Args:
        problem: LaCAM 問題。
        backend: 実 solver backend。未指定時は placeholder。
        time_limit_sec: 計算時間制限 [sec]。
        debug: デバッグ出力有効化フラグ。
        seed: 乱数シード。

    Returns:
        LaCAM 解。
    """
    runner = LaCAMRunner(
        backend=backend,
        config=LaCAMRunnerConfig(
            time_limit_sec=time_limit_sec,
            debug=debug,
            seed=seed,
        ),
    )
    return runner.solve(problem)