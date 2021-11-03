from pathlib import Path

from prompt_toolkit import PromptSession, prompt
from prompt_toolkit.completion import FuzzyCompleter, PathCompleter
from prompt_toolkit.validation import Validator


def is_valid_dir(text: Path):
    return text.exists() and text.is_dir()

def validate_in_range(value, valid_range):
    return value.isdigit() and int(value) in valid_range

dir_validator = Validator.from_callable(
    is_valid_dir,
    error_message="Not a valid directory (doesn't exist or is not a dir).",
    move_cursor_to_end=True,
)


def ask_for_path(initial):
    session = PromptSession(u"> ", completer=FuzzyCompleter(PathCompleter()))
    response = Path(session.prompt("Path to find new components\n> ",
                                   complete_while_typing=True,
                                   default=initial,
                                   pre_run=session.default_buffer.start_completion,
                                   validator=dir_validator))
    return response


def ask_for_path_selection(components_dir_list):
    print("Options")
    print("[0] .")
    for index, option in enumerate(components_dir_list):
        print(f"[{index + 1}] {option.path}")

    validator = Validator.from_callable(
        lambda x: validate_in_range(x, list(range(0, len(components_dir_list) + 1))),
        error_message=f'This input contains non valid number [0-{len(components_dir_list)}]',
        move_cursor_to_end=True)
    try:
        selected = prompt('> ',
                          validator=validator)
        if selected == "0":
            return Path.cwd()
        else:
            return components_dir_list[int(selected) - 1]
    except KeyboardInterrupt:
        return None
