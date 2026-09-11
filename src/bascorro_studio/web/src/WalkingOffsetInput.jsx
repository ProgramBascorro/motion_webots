import { useEffect, useRef, useState } from "react";
import { walkingOffsetFromDisplay, walkingOffsetToDisplay } from "./walkingPoseReference.js";

export default function WalkingOffsetInput({ value, referenceOffset, onChange, ...inputProps }) {
  const [text, setText] = useState(() => String(walkingOffsetToDisplay(value, referenceOffset) ?? ""));
  const lastEdit = useRef(null);

  useEffect(() => {
    if (lastEdit.current
      && Object.is(lastEdit.current.value, value)
      && lastEdit.current.referenceOffset === referenceOffset) return;
    lastEdit.current = null;
    setText(String(walkingOffsetToDisplay(value, referenceOffset) ?? ""));
  }, [value, referenceOffset]);

  return (
    <input
      {...inputProps}
      type="text"
      inputMode="decimal"
      value={text}
      onChange={(event) => {
        const entered = event.target.value;
        const native = walkingOffsetFromDisplay(entered, referenceOffset);
        lastEdit.current = { value: native, referenceOffset };
        setText(entered);
        onChange(native);
      }}
      onBlur={() => {
        lastEdit.current = null;
        setText(String(walkingOffsetToDisplay(value, referenceOffset) ?? ""));
      }}
    />
  );
}
