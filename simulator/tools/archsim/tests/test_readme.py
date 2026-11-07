"""The README's vocabulary cannot drift from inputs.json (which is transcribed from PLAN 3.3 / 3.5)."""
import pathlib
import re

README = (pathlib.Path(__file__).resolve().parents[1] / "README.md").read_text()


def test_every_vocab_term_in_readme(data):
    for kind in ("methods", "statuses", "classes"):
        for t in data["vocab"][kind]:
            assert "`%s`" % t["id"] in README, (kind, t["id"])


def test_readme_methods_table_matches_data(data):
    sec = README.split("| method | meaning |")[1].split("| status |")[0]
    assert set(re.findall(r"^\| `(\w+)` \|", sec, re.M)) == {t["id"] for t in data["vocab"]["methods"]}
    sec = README.split("| status | meaning |")[1].split("Calibration classes")[0]
    assert set(re.findall(r"^\| `(\w+)` \|", sec, re.M)) == {t["id"] for t in data["vocab"]["statuses"]}


def test_not_modelled_and_commands(data):
    for i in data["not_modelled"]["items"]:
        assert i["text"].split(" (")[0].lower().split(" beyond")[0] in README.lower(), i["text"]
    for cmd in ("make ui-archsim", "tools/archsim/build_html.py", "pytest tools/archsim/tests", "archsim.html"):
        assert cmd in README
