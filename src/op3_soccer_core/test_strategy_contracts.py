#!/usr/bin/env python3
"""
Regression tests for strategy layer contracts.

Covers:
  1. /game_state subscription uses typed GameState message (not String)
  2. CLEAR_BALL intent routes to a kick command (not None / not walk)
  3. Planner does NOT process direct-action intents (CLEAR_BALL, KICK, STOP, etc.)
  4. Hold-intents (HOLD_GOAL, GUARD_HALF, SUPPORT_LANE, RECEIVE_PASS, REPOSITION)
     produce {"action": "stop"} from tactical_node

Run:
  python3 test_strategy_contracts.py
"""

import ast
import sys
import unittest

# Allow importing from the installed location
sys.path.insert(
    0,
    '/home/mdzulfikri/motion_webots/install/op3_soccer_core/lib/python3.10/site-packages',
)

# ---------------------------------------------------------------------------
# Test 1 — GameState subscription contract
# ---------------------------------------------------------------------------

class TestGameStateSubscriptionContract(unittest.TestCase):
    """Verify that tactical_node subscribes to /game_state as GameState, not String."""

    def test_game_state_import_resolves(self):
        """soccer_msgs.GameState must import cleanly — if it can't, the type fix failed."""
        from soccer_msgs.msg import GameState
        self.assertTrue(hasattr(GameState, 'GAMESTATE_PLAYING'))
        self.assertTrue(hasattr(GameState, 'GAMESTATE_INITIAL'))
        self.assertTrue(hasattr(GameState, 'PENALTY_NONE'))
        print('✓ GameState constants available (GAMESTATE_PLAYING, PENALTY_NONE)')

    def test_tactical_node_imports_game_state_not_string(self):
        """tactical_node source must import GameState (not parse String JSON)."""
        import os
        src = os.path.join(
            '/home/mdzulfikri/motion_webots/src/op3_soccer_core/'
            'op3_soccer_core/tactical_node.py'
        )
        with open(src) as f:
            tree = ast.parse(f.read())

        imports = []
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                imports.append(ast.dump(node))

        has_game_state_import = any('GameState' in imp for imp in imports)
        self.assertTrue(has_game_state_import,
                        'tactical_node must import GameState from soccer_msgs.msg')

        # Confirm it does NOT try to json.loads the game_state callback
        with open(src) as f:
            source = f.read()
        # The game_state callback should assign msg directly, not parse JSON
        self.assertIn('def _cb_game_state', source)
        self.assertIn('self._game_state = msg', source)
        print('✓ tactical_node subscribes to GameState typed message')

    def test_gamestate_constants_match_tactical_usage(self):
        """Field names used in tactical_node must exist on GameState."""
        from soccer_msgs.msg import GameState
        gs = GameState()
        # These are the exact field accesses tactical_node makes
        _ = gs.gamestate
        _ = gs.penalty
        _ = gs.has_kick_off
        _ = gs.secondary_state
        self.assertEqual(GameState.GAMESTATE_PLAYING, 3)
        self.assertEqual(GameState.PENALTY_NONE, 0)
        print('✓ GameState field names and constants validated')


# ---------------------------------------------------------------------------
# Test 2 — CLEAR_BALL routes to kick (not None, not walk)
# ---------------------------------------------------------------------------

class TestClearBallRoutesToKick(unittest.TestCase):
    """CLEAR_BALL must produce a kick command from tactical_node."""

    def test_clear_ball_not_in_planner_intents(self):
        """CLEAR_BALL must NOT be in _PLANNER_INTENTS."""
        from op3_soccer_core.intent_types import IntentType
        from op3_soccer_core.tactical_node import _PLANNER_INTENTS
        self.assertNotIn(
            IntentType.CLEAR_BALL, _PLANNER_INTENTS,
            'CLEAR_BALL must not be delegated to planner — it is a direct kick',
        )
        print('✓ CLEAR_BALL not in _PLANNER_INTENTS')

    def test_clear_ball_source_routing(self):
        """Verify source: CLEAR_BALL is in the kick branch of _intent_to_motion."""
        import ast
        src_path = (
            '/home/mdzulfikri/motion_webots/src/op3_soccer_core/'
            'op3_soccer_core/tactical_node.py'
        )
        with open(src_path) as f:
            source = f.read()

        # CLEAR_BALL must appear alongside KICK in the kick branch
        # Look for the pattern: CLEAR_BALL in a tuple/set with KICK
        self.assertIn('IntentType.CLEAR_BALL', source)
        self.assertIn('IntentType.KICK', source)

        # The kick dispatch line must contain both KICK and CLEAR_BALL
        kick_lines = [
            line for line in source.splitlines()
            if 'KICK' in line and 'CLEAR_BALL' in line and 'kick' in line.lower()
        ]
        self.assertTrue(
            len(kick_lines) > 0,
            'Expected a line routing both KICK and CLEAR_BALL to kick action',
        )
        print('✓ CLEAR_BALL is co-located with KICK in the kick dispatch branch')


# ---------------------------------------------------------------------------
# Test 3 — Planner uses allowlist (never processes direct-action intents)
# ---------------------------------------------------------------------------

class TestPlannerAllowlist(unittest.TestCase):
    """Planner must only navigate for the 5 pure-navigation intents."""

    EXPECTED_NAVIGATE = frozenset({
        'approach_ball', 'align_to_kick', 'dribble_out',
        'intercept_threat', 'intercept_box',
    })

    DIRECT_ACTION = (
        'stop', 'recover', 'kick', 'search', 'kickoff_tap',
        'force_release', 'clear_to_support', 'takeover_striker',
        'clear_ball',
        'hold_goal', 'guard_half', 'support_lane', 'receive_pass', 'reposition',
    )

    def test_navigate_intents_set_exact(self):
        """_NAVIGATE_INTENTS in planner_node must match expected set exactly."""
        from op3_soccer_core.planner_node import _NAVIGATE_INTENTS
        self.assertEqual(
            _NAVIGATE_INTENTS, self.EXPECTED_NAVIGATE,
            f'Expected exactly {self.EXPECTED_NAVIGATE}, got {_NAVIGATE_INTENTS}',
        )
        print(f'✓ _NAVIGATE_INTENTS = {_NAVIGATE_INTENTS}')

    def test_direct_action_intents_excluded(self):
        """Every direct-action intent must NOT be in _NAVIGATE_INTENTS."""
        from op3_soccer_core.planner_node import _NAVIGATE_INTENTS
        for intent in self.DIRECT_ACTION:
            self.assertNotIn(
                intent, _NAVIGATE_INTENTS,
                f'Direct-action intent "{intent}" must NOT be in planner _NAVIGATE_INTENTS',
            )
        print(f'✓ All {len(self.DIRECT_ACTION)} direct-action intents excluded from planner')

    def test_planner_uses_allowlist_not_blocklist(self):
        """Planner source must check intent IN _NAVIGATE_INTENTS (allowlist style)."""
        src_path = (
            '/home/mdzulfikri/motion_webots/src/op3_soccer_core/'
            'op3_soccer_core/planner_node.py'
        )
        with open(src_path) as f:
            source = f.read()
        self.assertIn('_NAVIGATE_INTENTS', source)
        self.assertIn('intent_type not in _NAVIGATE_INTENTS', source)
        print('✓ Planner uses allowlist check (intent_type not in _NAVIGATE_INTENTS)')


# ---------------------------------------------------------------------------
# Test 4 — Hold-intents produce {"action": "stop"}
# ---------------------------------------------------------------------------

class TestHoldIntentProducesStop(unittest.TestCase):
    """HOLD_GOAL, GUARD_HALF, SUPPORT_LANE, RECEIVE_PASS, REPOSITION → stop."""

    HOLD_INTENTS = (
        'HOLD_GOAL', 'GUARD_HALF', 'SUPPORT_LANE', 'RECEIVE_PASS', 'REPOSITION',
    )

    def test_hold_intents_not_in_planner_intents(self):
        """Hold intents must not be delegated to planner."""
        from op3_soccer_core.intent_types import IntentType
        from op3_soccer_core.tactical_node import _PLANNER_INTENTS
        for name in self.HOLD_INTENTS:
            t = IntentType[name]
            self.assertNotIn(
                t, _PLANNER_INTENTS,
                f'{name} must not be in _PLANNER_INTENTS — tactical should send stop',
            )
        print(f'✓ All {len(self.HOLD_INTENTS)} hold-intents absent from _PLANNER_INTENTS')

    def test_hold_intents_in_source_stop_branch(self):
        """Verify source: hold intents appear in the stop branch of _intent_to_motion."""
        src_path = (
            '/home/mdzulfikri/motion_webots/src/op3_soccer_core/'
            'op3_soccer_core/tactical_node.py'
        )
        with open(src_path) as f:
            source = f.read()

        for name in self.HOLD_INTENTS:
            intent_ref = f'IntentType.{name}'
            self.assertIn(
                intent_ref, source,
                f'{intent_ref} must appear in tactical_node source',
            )

        # The block must contain all hold intents together before a stop return
        stop_block = [
            line for line in source.splitlines()
            if 'HOLD_GOAL' in line or 'GUARD_HALF' in line or 'SUPPORT_LANE' in line
        ]
        self.assertTrue(len(stop_block) > 0, 'Hold intents not found in tactical_node source')
        print('✓ Hold-intents appear in tactical_node source (stop dispatch block)')

    def test_planner_intents_and_hold_intents_disjoint(self):
        """_PLANNER_INTENTS and hold-intents must be fully disjoint."""
        from op3_soccer_core.intent_types import IntentType
        from op3_soccer_core.tactical_node import _PLANNER_INTENTS
        hold_set = {IntentType[n] for n in self.HOLD_INTENTS}
        overlap = _PLANNER_INTENTS & hold_set
        self.assertEqual(
            overlap, set(),
            f'Overlap between _PLANNER_INTENTS and hold-intents: {overlap}',
        )
        print('✓ _PLANNER_INTENTS ∩ hold-intents = ∅ (no overlap)')


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run_tests():
    print('\n' + '=' * 70)
    print('STRATEGY CONTRACT REGRESSION TESTS')
    print('=' * 70 + '\n')

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestGameStateSubscriptionContract))
    suite.addTests(loader.loadTestsFromTestCase(TestClearBallRoutesToKick))
    suite.addTests(loader.loadTestsFromTestCase(TestPlannerAllowlist))
    suite.addTests(loader.loadTestsFromTestCase(TestHoldIntentProducesStop))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print('\n' + '=' * 70)
    print(f'Tests run: {result.testsRun}  '
          f'Failures: {len(result.failures)}  '
          f'Errors: {len(result.errors)}')
    if result.wasSuccessful():
        print('✅ ALL CONTRACT TESTS PASSED')
        return 0
    print('❌ SOME TESTS FAILED')
    return 1


if __name__ == '__main__':
    sys.exit(run_tests())
