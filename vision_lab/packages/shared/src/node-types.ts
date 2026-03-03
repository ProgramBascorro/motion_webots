import type { OutputKind } from "./graph-schema";

export type ParameterFieldType = "text" | "number" | "boolean" | "select" | "string_array" | "int_array";

export type ParameterField = {
  key: string;
  label: string;
  type: ParameterFieldType;
  required?: boolean;
  defaultValue?: unknown;
  min?: number;
  max?: number;
  step?: number;
  options?: Array<{ label: string; value: string | number }>;
  helperText?: string;
};

export type NodeCategory = "input" | "preprocess" | "inference" | "postprocess";

export type NodeTypeDefinition = {
  type: string;
  displayName: string;
  category: NodeCategory;
  acceptedInputKinds: OutputKind[];
  producedOutputKinds: OutputKind[];
  paramsSchema: ParameterField[];
  previewable: boolean;
  maxInputs: number;
};
